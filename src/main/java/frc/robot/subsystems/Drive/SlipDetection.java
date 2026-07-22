package frc.robot.subsystems.Drive;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.MetersPerSecondPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.DriveConstants;

/**
 * Detects wheel slip by comparing wheel-encoder-implied motion against
 * references the wheels can't influence. The combined score (0 = no slip,
 * 1 = fully slipping) is the highest of the available terms, so one strong
 * signal isn't diluted by others that currently have nothing to say. Only
 * the combined score/boolean matters for Glass — which term tripped it is
 * exposed separately purely for tuning, not as something to watch live.
 *
 * Terms:
 * - Rotational (always available): wheel-implied yaw rate vs. the gyro.
 *   This is a diagnostic only — the pose estimator's heading already comes
 *   directly from the gyro, never from wheel rotation, so this term can't
 *   protect pose accuracy the way the translational term does. It's still
 *   useful for flagging "wheels spinning oddly" (defense contact, a bound
 *   module) on its own.
 * - Translational, vision-backed (opportunistic): wheel-implied displacement
 *   vs. vision-measured displacement since the last vision fix. Feeds the
 *   thing that actually matters for pose accuracy.
 * - Translational, accelerometer fallback (always available, noisier):
 *   wheel-implied displacement vs. IMU-double-integrated displacement,
 *   compared once per accelFallbackWindowSeconds. Used only when vision
 *   hasn't reported anything fresh — this is how we still catch "wheels
 *   spinning, robot pinned" with no tags in view. Comparing displacement
 *   over a window rather than instantaneous acceleration is deliberate: a
 *   pure acceleration comparison goes quiet the instant commanded velocity
 *   stops changing (both sides read ~0 whether cruising or stuck), which
 *   is exactly why an earlier version of this only caught slip for a single
 *   instant instead of for as long as it was actually happening.
 *
 * Stall is tracked separately from slip (its own score/boolean/Glass
 * indicator, never folded into getSlipScore()) — it's a different physical
 * failure mode. A module commanded to move but physically blocked (e.g. full
 * wheel grip against a wall) has its OWN encoder agreeing nothing is
 * happening, which is exactly why none of the slip terms above can see it —
 * they only notice when wheel-implied motion disagrees with some other
 * reference, and a stalled wheel's own encoder isn't lying about anything.
 */
public class SlipDetection {

    private double rotationalSlip = 0.0;
    private double translationalSlipVision = 0.0;
    private double translationalSlipAccel = 0.0;
    private double stallScore = 0.0;

    private double lastLoopTimestamp = -1;
    private double lastVisionUpdateTimestamp = -1;

    // Wheel-implied displacement accumulated since the last vision fix —
    // reset to zero every time a fresh vision fix arrives, so this always
    // reflects only the most recent short window, not lifetime drift.
    private double wheelDriftX = 0.0;
    private double wheelDriftY = 0.0;
    private Translation2d lastVisionPosition = null;

    // Accelerometer fallback: double-integrated IMU displacement and
    // wheel-implied displacement, both accumulated over a fixed window and
    // compared/reset on a timer (not tied to vision arrival at all).
    private double accelWindowStartTime = -1;
    private double imuVx = 0.0;
    private double imuVy = 0.0;
    private double imuDriftX = 0.0;
    private double imuDriftY = 0.0;
    private double wheelDriftAccelX = 0.0;
    private double wheelDriftAccelY = 0.0;

    /** Called once per loop from the drivetrain's periodic. */
    public void update(CommandSwerveDrivetrain drivetrain) {
        double now = Timer.getFPGATimestamp();
        double dt = (lastLoopTimestamp < 0) ? 0.02 : now - lastLoopTimestamp;
        lastLoopTimestamp = now;

        updateRotationalSlip(drivetrain);
        updateWheelDrift(drivetrain, dt);
        updateAccelFallback(drivetrain, dt);
        updateStall(drivetrain);
        logValues();
    }

    /**
     * Wheel odometry and the gyro both measure yaw rate independently — if
     * they disagree, the wheels are turning without the chassis actually
     * turning that fast (or at all).
     */
    private void updateRotationalSlip(CommandSwerveDrivetrain drivetrain) {
        double wheelOmegaRadPerSec = drivetrain.getState().Speeds.omegaRadiansPerSecond;
        double gyroOmegaRadPerSec = drivetrain.getPigeon2().getAngularVelocityZWorld().getValue().in(RadiansPerSecond);

        double error = Math.abs(wheelOmegaRadPerSec - gyroOmegaRadPerSec);
        rotationalSlip = MathUtil.clamp(error / DriveConstants.rotationalSlipThresholdRadPerSec, 0.0, 1.0);
    }

    /** Integrates field-relative wheel speed to track displacement between vision fixes. */
    private void updateWheelDrift(CommandSwerveDrivetrain drivetrain, double dt) {
        ChassisSpeeds fieldSpeeds = drivetrain.getFieldSpeeds();
        wheelDriftX += fieldSpeeds.vxMetersPerSecond * dt;
        wheelDriftY += fieldSpeeds.vyMetersPerSecond * dt;
    }

    /**
     * Called from Vision whenever a trustworthy pose estimate lands (skip
     * rejected/very-low-confidence fixes — they'd just inject noise here).
     * Compares vision's measured displacement since the last fix against
     * what the wheels claimed over that same window.
     */
    public void reportVisionUpdate(Translation2d visionPosition) {
        if (lastVisionPosition != null) {
            double visionDx = visionPosition.getX() - lastVisionPosition.getX();
            double visionDy = visionPosition.getY() - lastVisionPosition.getY();

            double errorX = wheelDriftX - visionDx;
            double errorY = wheelDriftY - visionDy;
            double error = Math.hypot(errorX, errorY);

            translationalSlipVision = MathUtil.clamp(error / DriveConstants.translationalSlipThresholdMeters, 0.0, 1.0);
        }

        lastVisionPosition = visionPosition;
        lastVisionUpdateTimestamp = Timer.getFPGATimestamp();
        wheelDriftX = 0.0;
        wheelDriftY = 0.0;
    }

    /**
     * No-vision translational fallback. Every loop, both the accelerometer
     * (double-integrated: accel -> velocity -> displacement) and the wheels
     * (velocity -> displacement) accumulate their own idea of how far the
     * robot has moved. Once per accelFallbackWindowSeconds, the two are
     * compared and both accumulators reset for the next window — this is
     * what lets a sustained mismatch (not just the initial transient) keep
     * reporting slip, at the cost of some accelerometer drift.
     *
     * The IMU velocity estimate is re-synced to the wheel-implied velocity
     * at the end of any window where the two already agreed — using
     * disagreement itself as the reason not to trust the wheels as a
     * recalibration source. This keeps long-term accelerometer drift
     * bounded during normal driving without erasing an active, ongoing
     * slip event the moment it's detected.
     */
    private void updateAccelFallback(CommandSwerveDrivetrain drivetrain, double dt) {
        double imuAx = drivetrain.getPigeon2().getAccelerationX().getValue().in(MetersPerSecondPerSecond);
        double imuAy = drivetrain.getPigeon2().getAccelerationY().getValue().in(MetersPerSecondPerSecond);
        imuVx += imuAx * dt;
        imuVy += imuAy * dt;
        imuDriftX += imuVx * dt;
        imuDriftY += imuVy * dt;

        ChassisSpeeds robotSpeeds = drivetrain.getState().Speeds;
        wheelDriftAccelX += robotSpeeds.vxMetersPerSecond * dt;
        wheelDriftAccelY += robotSpeeds.vyMetersPerSecond * dt;

        double now = Timer.getFPGATimestamp();
        if (accelWindowStartTime < 0) {
            accelWindowStartTime = now;
        }
        if (now - accelWindowStartTime < DriveConstants.accelFallbackWindowSeconds) {
            return;
        }

        double error = Math.hypot(wheelDriftAccelX - imuDriftX, wheelDriftAccelY - imuDriftY);
        translationalSlipAccel = MathUtil.clamp(error / DriveConstants.translationalAccelDriftThresholdMeters, 0.0, 1.0);

        if (translationalSlipAccel <= DriveConstants.slipDetectedThreshold) {
            imuVx = robotSpeeds.vxMetersPerSecond;
            imuVy = robotSpeeds.vyMetersPerSecond;
        }

        imuDriftX = 0.0;
        imuDriftY = 0.0;
        wheelDriftAccelX = 0.0;
        wheelDriftAccelY = 0.0;
        accelWindowStartTime = now;
    }

    /**
     * Per-module commanded vs. actual velocity, corroborated with drive
     * current so a low-speed module that's simply not being asked to move
     * fast doesn't get flagged. Reports the worst of the four modules.
     */
    private void updateStall(CommandSwerveDrivetrain drivetrain) {
        var state = drivetrain.getState();
        double worstStall = 0.0;

        for (int i = 0; i < state.ModuleStates.length; i++) {
            double commanded = Math.abs(state.ModuleTargets[i].speedMetersPerSecond);
            if (commanded < DriveConstants.stallMinCommandedVelocityMPS) {
                continue;
            }

            double current = drivetrain.getModule(i).getDriveMotor().getStatorCurrent().getValue().in(Amps);
            if (current < DriveConstants.stallCurrentThresholdAmps) {
                continue;
            }

            double actual = Math.abs(state.ModuleStates[i].speedMetersPerSecond);
            double deficitRatio = (commanded - actual) / commanded;
            double moduleStall = MathUtil.clamp(deficitRatio / DriveConstants.stallVelocityDeficitRatioThreshold, 0.0, 1.0);
            worstStall = Math.max(worstStall, moduleStall);
        }

        stallScore = worstStall;
    }

    public double getStallScore() {
        return stallScore;
    }

    /** For its own Glass Boolean Box, separate from SLIP/Slipping. */
    public boolean isStalled() {
        return stallScore > DriveConstants.stallDetectedThreshold;
    }

    public boolean isVisionFresh() {
        return lastVisionUpdateTimestamp > 0
                && (Timer.getFPGATimestamp() - lastVisionUpdateTimestamp) < DriveConstants.visionFreshnessWindowSeconds;
    }

    private double getTranslationalSlip() {
        return isVisionFresh() ? translationalSlipVision : translationalSlipAccel;
    }

    public double getSlipScore() {
        return Math.max(rotationalSlip, getTranslationalSlip());
    }

    /** The one number Glass should actually watch — true once the combined score crosses the detection threshold. */
    public boolean isSlipping() {
        return getSlipScore() > DriveConstants.slipDetectedThreshold;
    }

    private void logValues() {
        SmartDashboard.putNumber("SLIP/Score", getSlipScore());
        SmartDashboard.putBoolean("SLIP/Slipping", isSlipping());
        SmartDashboard.putNumber("STALL/Score", getStallScore());
        SmartDashboard.putBoolean("STALL/Stalled", isStalled());

        // Tuning-only telemetry, not meant for the driver-facing indicators.
        SmartDashboard.putNumber("SLIP/Debug/Rotational", rotationalSlip);
        SmartDashboard.putNumber("SLIP/Debug/TranslationalVision", translationalSlipVision);
        SmartDashboard.putNumber("SLIP/Debug/TranslationalAccel", translationalSlipAccel);
        SmartDashboard.putBoolean("SLIP/Debug/VisionFresh", isVisionFresh());
    }
}
