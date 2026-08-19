// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.Meters;

import java.util.Map;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.util.Color;

/**
 * Robot-wide constants. All values are public static; nothing in this class
 * should be functional.
 *
 * NOTE: alliance color must always be read live via
 * DriverStation.getAlliance() — never cache it in a static field, since it is
 * empty until the Driver Station connects.
 */
public final class Constants {

    public static class OperatorConstants {
        public static final int kDriverControllerPort = 0;
        public static final int kOperatorControllerPort = 1;
    }

    public static class DriveConstants {
        // Mismatch between wheel-implied yaw rate and the gyro's measured
        // yaw rate, in rad/s, above which we call it rotational slip.
        // TODO: tune against real driving/defense contact.
        public static final double rotationalSlipThresholdRadPerSec = 1.0;

        // Position error, in meters, between wheel-implied displacement and
        // vision-measured displacement over the same interval, above which
        // we call it translational slip. TODO: tune.
        public static final double translationalSlipThresholdMeters = 0.3;

        // How often the accelerometer fallback compares accumulated
        // wheel-implied displacement against accumulated IMU-integrated
        // displacement. Comparing raw acceleration alone goes quiet the
        // instant commanded velocity stops changing (both sides read ~0
        // whether cruising or stuck) — comparing displacement over a short
        // window instead keeps flagging a sustained mismatch, not just the
        // initial transient.
        public static final double accelFallbackWindowSeconds = 0.5;

        // Position error, in meters, between wheel-implied and IMU-integrated
        // displacement over one accelFallbackWindowSeconds window, above
        // which the accelerometer fallback calls it slip. Looser than the
        // vision threshold since double-integrated accelerometer data is
        // much noisier. TODO: tune.
        public static final double translationalAccelDriftThresholdMeters = 0.5;

        // How long a vision fix stays "fresh" enough to drive the
        // translational term before falling back to the accelerometer.
        public static final double visionFreshnessWindowSeconds = 0.5;

        // Combined slip score (0-1) above which the SLIP/Slipping indicator
        // reports true.
        public static final double slipDetectedThreshold = 0.5;

        // Stall detection — a module commanded to move but physically
        // blocked (e.g. driving into a wall with full wheel grip), distinct
        // from slip: the wheel's own encoder agrees nothing is happening,
        // which is why the slip terms above can't catch this case.
        // TODO: tune all of these against real driving/wall contact.

        // Below this commanded speed we don't check for stall at all —
        // there's nothing meaningful to compare against near-zero commands.
        public static final double stallMinCommandedVelocityMPS = 0.5;

        // Drive motor stator current, in amps, above which a module is
        // considered to be "pushing hard" rather than just idly ramping up.
        // Drive motors are current-limited to 60A (TunerConstants) — this
        // should sit comfortably below that.
        public static final double stallCurrentThresholdAmps = 40.0;

        // Fraction of commanded velocity a module is missing (0-1) before
        // we call it fully stalled.
        public static final double stallVelocityDeficitRatioThreshold = 0.7;

        // Stall score (0-1) above which the STALL/Stalled indicator reports true.
        public static final double stallDetectedThreshold = 0.5;
    }

    public static class AutoConstants {
        // Pathfinding constraints, deliberately set above the drivetrain's
        // physical limits so the robot's own kinematics — not these caps —
        // are what bounds pathfindToPose, validated at competition.
        public static final double kMaxSpeedMetersPerSecond = 16;
        public static final double kMaxAccelerationMetersPerSecondSquared = 100;
        public static final double kMaxAngularSpeedRadiansPerSecond = 100;
        public static final double kMaxAngularSpeedRadiansPerSecondSquared = 100;

        public static final double xTolerance = 0.05;
        public static final double yTolerance = 0.05;
        public static final double rotTolerance = Units.degreesToRadians(5);
    }

    public static class ShooterConstants {
        public static final double hoodConversionRotToDeg = 360 / 129.6;
        public static final double MIN_RPS = 0;
        public static final double MAX_RPS = 90;

        /*
         * Tuned shot tables (distance to hub in meters -> value), as of
         * Niagara — the later ONCMP-event revisions to these tables were
         * attempts to compensate for a mechanical issue rather than genuine
         * tuning improvements, so they're intentionally not used here.
         */
        public static final InterpolatingDoubleTreeMap RPS_MAP = new InterpolatingDoubleTreeMap();
        static {
            RPS_MAP.put(2.0, 45.5d);
            RPS_MAP.put(2.5, 47.5d);
            RPS_MAP.put(3.0, 48d);
            RPS_MAP.put(3.5, 50d);
            RPS_MAP.put(4.0, 52d);
            RPS_MAP.put(4.5, 54.5d);
            RPS_MAP.put(5.0, 56d);
            RPS_MAP.put(5.5, 60d);
            RPS_MAP.put(6.0, 64d);
        }

        public static final InterpolatingDoubleTreeMap HOOD_MAP = new InterpolatingDoubleTreeMap();
        static {
            HOOD_MAP.put(2.0, 0d);
            HOOD_MAP.put(2.5, 1d);
            HOOD_MAP.put(3.0, 2.2d);
            HOOD_MAP.put(3.5, 3d);
            HOOD_MAP.put(4.0, 3.8d);
            HOOD_MAP.put(4.5, 4.3d);
            HOOD_MAP.put(5.0, 4.6d);
            HOOD_MAP.put(5.5, 5d);
            HOOD_MAP.put(6.0, 5.5d);
        }

        // Time of flight (seconds) by distance. Feeds the SOTF virtual-target
        // lead (v * tof), so errors here directly under/over-correct moving
        // shots.
        public static final InterpolatingDoubleTreeMap TOF_MAP = new InterpolatingDoubleTreeMap();
        static {
            TOF_MAP.put(2.0, 0.5);
            TOF_MAP.put(3.0, 0.5);
            TOF_MAP.put(4.0, 0.5);
            TOF_MAP.put(5.0, 0.5);
            TOF_MAP.put(6.0, 0.5);
        }

        // Passing shot tables, tuned on Programming-Points at ONCMP/Niagara.
        public static final InterpolatingDoubleTreeMap PASSING_TOF_MAP = new InterpolatingDoubleTreeMap();
        static {
            PASSING_TOF_MAP.put(2.0, 0.5d);
            PASSING_TOF_MAP.put(2.5, 0.5d);
            PASSING_TOF_MAP.put(3.0, 0.5d);
            PASSING_TOF_MAP.put(3.5, 0.5d);
            PASSING_TOF_MAP.put(4.0, 0.5d);
            PASSING_TOF_MAP.put(4.5, 0.5d);
            PASSING_TOF_MAP.put(5.0, 0.5d);
            PASSING_TOF_MAP.put(5.5, 0.5d);
            PASSING_TOF_MAP.put(6.0, 0.5d);
            PASSING_TOF_MAP.put(7.0, 0.5d);
            PASSING_TOF_MAP.put(8.0, 0.5d);
            PASSING_TOF_MAP.put(9.0, 0.5d);
            PASSING_TOF_MAP.put(10.0, 0.5d);
            PASSING_TOF_MAP.put(11.0, 0.5d);
            PASSING_TOF_MAP.put(12.0, 0.5d);
            PASSING_TOF_MAP.put(13.0, 0.5d);
        }

        public static final InterpolatingDoubleTreeMap PASSING_HOOD_MAP = new InterpolatingDoubleTreeMap();
        static {
            PASSING_HOOD_MAP.put(2.0, 7.5d);
            PASSING_HOOD_MAP.put(2.5, 7.5d);
            PASSING_HOOD_MAP.put(3.0, 7.5d);
            PASSING_HOOD_MAP.put(3.5, 7.5d);
            PASSING_HOOD_MAP.put(4.0, 7.5d);
            PASSING_HOOD_MAP.put(4.5, 7.5d);
            PASSING_HOOD_MAP.put(5.0, 7.5d);
            PASSING_HOOD_MAP.put(5.5, 7.5d);
            PASSING_HOOD_MAP.put(6.0, 7.5d);
            PASSING_HOOD_MAP.put(7.0, 7.5d);
            PASSING_HOOD_MAP.put(8.0, 8.0d);
            PASSING_HOOD_MAP.put(9.0, 8.0d);
            PASSING_HOOD_MAP.put(10.0, 8.0d);
            PASSING_HOOD_MAP.put(11.0, 8.0d);
            PASSING_HOOD_MAP.put(12.0, 8.0d);
            PASSING_HOOD_MAP.put(13.0, 8.0d);
        }

        public static final InterpolatingDoubleTreeMap PASSING_RPS_MAP = new InterpolatingDoubleTreeMap();
        static {
            PASSING_RPS_MAP.put(2.0, 33d);
            PASSING_RPS_MAP.put(2.5, 35d);
            PASSING_RPS_MAP.put(3.0, 37d);
            PASSING_RPS_MAP.put(3.5, 40d);
            PASSING_RPS_MAP.put(4.0, 42d);
            PASSING_RPS_MAP.put(5.0, 46d);
            PASSING_RPS_MAP.put(6.0, 51d);
            PASSING_RPS_MAP.put(7.0, 57d);
            PASSING_RPS_MAP.put(8.0, 63d);
            PASSING_RPS_MAP.put(9.0, 67d);
            PASSING_RPS_MAP.put(10.0, 80d);
            PASSING_RPS_MAP.put(11.0, 83d);
            PASSING_RPS_MAP.put(12.0, 86d);
            PASSING_RPS_MAP.put(13.0, 90d);
        }

        /*
         * Mid-match shot-profile quick-fix. Switches which pre-baked RPS/HOOD
         * table ShotCalc reads from, to compensate for a suspected external
         * calibration bias (e.g. field lighting throwing off vision-based
         * distance) that isn't a mechanical or tuning problem -- without
         * touching the validated NORMAL tables above.
         *
         * LONG_SHOT/SHORT_SHOT are meant to be independently practice-tuned,
         * the same way NORMAL was: NORMAL swishes center goal, LONG_SHOT is
         * tuned to land on the back edge, SHORT_SHOT on the front edge. That
         * anchors each profile's correction to the goal's own real tolerance
         * instead of a guessed percentage, and lets the correction's size
         * vary by distance instead of being a flat scalar. PLACEHOLDER: not
         * yet re-tuned -- currently just a copy of NORMAL below. Needs real
         * practice-field edge-shot passes before this is trustworthy at a
         * competition; edit each .put() line directly with the real value
         * once measured, same as NORMAL's tables were built.
         */
        public enum ShotProfile {
            SHORT_SHOT, NORMAL, LONG_SHOT
        }

        public static ShotProfile activeShotProfile = ShotProfile.NORMAL;

        public static final InterpolatingDoubleTreeMap RPS_MAP_LONG_SHOT = new InterpolatingDoubleTreeMap();
        static {
            RPS_MAP_LONG_SHOT.put(2.0, 45.5d);
            RPS_MAP_LONG_SHOT.put(2.5, 47.5d);
            RPS_MAP_LONG_SHOT.put(3.0, 48d);
            RPS_MAP_LONG_SHOT.put(3.5, 50d);
            RPS_MAP_LONG_SHOT.put(4.0, 52d);
            RPS_MAP_LONG_SHOT.put(4.5, 54.5d);
            RPS_MAP_LONG_SHOT.put(5.0, 56d);
            RPS_MAP_LONG_SHOT.put(5.5, 60d);
            RPS_MAP_LONG_SHOT.put(6.0, 64d);
        }

        public static final InterpolatingDoubleTreeMap HOOD_MAP_LONG_SHOT = new InterpolatingDoubleTreeMap();
        static {
            HOOD_MAP_LONG_SHOT.put(2.0, 0d);
            HOOD_MAP_LONG_SHOT.put(2.5, 1d);
            HOOD_MAP_LONG_SHOT.put(3.0, 2.2d);
            HOOD_MAP_LONG_SHOT.put(3.5, 3d);
            HOOD_MAP_LONG_SHOT.put(4.0, 3.8d);
            HOOD_MAP_LONG_SHOT.put(4.5, 4.3d);
            HOOD_MAP_LONG_SHOT.put(5.0, 4.6d);
            HOOD_MAP_LONG_SHOT.put(5.5, 5d);
            HOOD_MAP_LONG_SHOT.put(6.0, 5.5d);
        }

        public static final InterpolatingDoubleTreeMap RPS_MAP_SHORT_SHOT = new InterpolatingDoubleTreeMap();
        static {
            RPS_MAP_SHORT_SHOT.put(2.0, 45.5d);
            RPS_MAP_SHORT_SHOT.put(2.5, 47.5d);
            RPS_MAP_SHORT_SHOT.put(3.0, 48d);
            RPS_MAP_SHORT_SHOT.put(3.5, 50d);
            RPS_MAP_SHORT_SHOT.put(4.0, 52d);
            RPS_MAP_SHORT_SHOT.put(4.5, 54.5d);
            RPS_MAP_SHORT_SHOT.put(5.0, 56d);
            RPS_MAP_SHORT_SHOT.put(5.5, 60d);
            RPS_MAP_SHORT_SHOT.put(6.0, 64d);
        }

        public static final InterpolatingDoubleTreeMap HOOD_MAP_SHORT_SHOT = new InterpolatingDoubleTreeMap();
        static {
            HOOD_MAP_SHORT_SHOT.put(2.0, 0d);
            HOOD_MAP_SHORT_SHOT.put(2.5, 1d);
            HOOD_MAP_SHORT_SHOT.put(3.0, 2.2d);
            HOOD_MAP_SHORT_SHOT.put(3.5, 3d);
            HOOD_MAP_SHORT_SHOT.put(4.0, 3.8d);
            HOOD_MAP_SHORT_SHOT.put(4.5, 4.3d);
            HOOD_MAP_SHORT_SHOT.put(5.0, 4.6d);
            HOOD_MAP_SHORT_SHOT.put(5.5, 5d);
            HOOD_MAP_SHORT_SHOT.put(6.0, 5.5d);
        }

        /** The RPS table ShotCalc should read for a stationary (non-passing) hub shot, per the active profile. */
        public static InterpolatingDoubleTreeMap activeRpsMap() {
            return switch (activeShotProfile) {
                case NORMAL -> RPS_MAP;
                case LONG_SHOT -> RPS_MAP_LONG_SHOT;
                case SHORT_SHOT -> RPS_MAP_SHORT_SHOT;
            };
        }

        /** The hood table ShotCalc should read for a stationary (non-passing) hub shot, per the active profile. */
        public static InterpolatingDoubleTreeMap activeHoodMap() {
            return switch (activeShotProfile) {
                case NORMAL -> HOOD_MAP;
                case LONG_SHOT -> HOOD_MAP_LONG_SHOT;
                case SHORT_SHOT -> HOOD_MAP_SHORT_SHOT;
            };
        }

        private static final ShotProfile[] SHOT_PROFILE_STEPS = { ShotProfile.SHORT_SHOT, ShotProfile.NORMAL,
                ShotProfile.LONG_SHOT };

        /** Steps the active profile one notch toward LONG_SHOT (no-op if already there). */
        public static void stepShotProfileTowardLong() {
            int index = java.util.Arrays.asList(SHOT_PROFILE_STEPS).indexOf(activeShotProfile);
            activeShotProfile = SHOT_PROFILE_STEPS[Math.min(index + 1, SHOT_PROFILE_STEPS.length - 1)];
        }

        /** Steps the active profile one notch toward SHORT_SHOT (no-op if already there). */
        public static void stepShotProfileTowardShort() {
            int index = java.util.Arrays.asList(SHOT_PROFILE_STEPS).indexOf(activeShotProfile);
            activeShotProfile = SHOT_PROFILE_STEPS[Math.max(index - 1, 0)];
        }

        /** Resets the quick-fix profile to NORMAL -- called on every disable so a correction from a prior match/field doesn't carry over. */
        public static void resetShotProfile() {
            activeShotProfile = ShotProfile.NORMAL;
        }

        /*
         * Historical polynomial-regression tuning data, superseded by the maps
         * above. Kept for reference:
         * hood:    (2,0), (3,2.5), (4,5), (5,5.5), (6,6.5), (10,8)     [deg 2]
         * rps:     (2,47), (3,50), (4,50), (5,55), (6,62), (10,85)     [deg 2]
         * tof:     (2,0.25), (3,0.30), (4,0.35), (5,0.40), (6,0.45)    [deg 2]
         */

        // Hood position clamp (mechanism rotations) -- the physical safe range
        // enforced on every hood setpoint, including manual debug control.
        public static final double hoodMinPosition = -0.5;
        public static final double hoodMaxPosition = 8;

        // Manual debug control: how far the manual hood/flywheel setpoints
        // move per second of full joystick deflection, and the deadband
        // applied to the stick before nudging either one.
        public static final double debugHoodRateRotPerSec = 3.0;
        public static final double debugRPSRatePerSec = 20.0;
        public static final double debugStickDeadband = 0.15;

        public static final double activeWaitingSpeed = 30;
        public static final double inactiveWaitingSpeed = 0;

        public static final double shooterMotionMagicExpoK_V = 0.1;
        public static final double shooterMotionMagicExpoK_A = 0.1;
        public static final double shooterMotionMagicAccel = 400; // rps^2
        public static final double shooterMotionMagicJerk = 4000; // rps^2/s

        public static final int SupplyCurrentLimit = 80;
        public static final int StatorCurrentLimit = 80;

        public static final int hoodSupplyCurrentLimit = 15;
        public static final int hoodStatorCurrentLimit = 15;

        // Supply current (amps) above which hood homing considers itself
        // against the hard stop. 0 disables auto-zeroing during HOMING
        // (the state will hold gently against the stop but never re-zero).
        public static final double homingThreshold = 13;

        // Flywheel is "ready" when actual speed is within this many RPS of
        // the setpoint.
        public static final double readyToleranceRPS = 3.5;

        public static final int hoodMotorID = 53;
        public static final int shooterMotor1ID = 61;
        public static final int shooterMotor2ID = 60;

        public static final double[] hoodPID = { 2.0, 0, 0 };
        public static final double[] hoodSVA = { 0.0, 0, 0 };

        // kP 1.5/kS 0.5 dated back to a July 13 baseline snapshot of main with no
        // recorded tuning session; bench-retuned via Tuner (Velocity Voltage, Slot 0)
        // after it was chattering/oscillating around target -- kP 0.2/kS 0.1 held flat.
        public static final double[] shooterPID = { 0.2, 0, 0.02 };
        public static final double[] shooterSVA = { 0.15, 0.117, 0.02 };

        public enum ShooterWantedState {
            IDLE,
            WAIT,
            TRENCH_SHOOT,
            PASS_SHOOT,
            HUB_SHOOT,
            HOME,
            TEST,
            RETRACT_AUTO,
            TURN_ON_AUTO,
            DEBUG
        }

        public enum SystemState {
            IDLING,
            ACTIVE_WAITING,
            INACTIVE_WAITING,
            TRENCH_SHOOTING,
            PASS_SHOOTING,
            HUB_SHOOTING,
            HOMING,
            TESTING,
            RETRACTING_AUTO,
            TURNING_ON_AUTO,
            DEBUGGING
        }
    }

    public static class IntakeConstants {
        public static final double intakeMotionMagicExpoK_V = 0.13;
        public static final double intakeMotionMagicExpoK_A = 0.1;

        public static final int SupplyCurrentLimit = 40;
        public static final int StatorCurrentLimit = 100;

        public static final int ExtensionSupplyCurrentLimit = 80;
        public static final int ExtensionStatorCurrentLimit = 50;

        public static final int intakeMotorID = 32;
        public static final int intakeMotor2ID = 34;
        public static final int intakeExtensionMotorID = 31;
        public static final int canRangeID = 33;

        public static final double intakingPosition = 10;
        public static final double intakingSpeed = -0.9;
        // Slower Motion Magic kA applied to the extension while SCORING
        // (slow squeeze), restored on leaving the state.
        public static final double slowerIntakeKa = 1.0;
        // CANrange distance (meters) beyond which the extension is considered
        // at home, used only by the (currently unbound) RESET auto-zero state.
        // 0 disables it. The intended re-zero workflow after a big impact is
        // manual: D-pad nudge the intake into the hard stop, then press the
        // driver's back button (setZero).
        public static final double intakeExtensionHomingThreshold = 0;
        public static final double retractingPos = 0;
        // Duty cycle used for manual nudging and homing crawl.
        // Must clear the extension TalonFX's DutyCycleNeutralDeadband
        // (Phoenix6 default 0.04 / 4%, never overridden in Intake's config)
        // or the motor outputs true zero and never moves. 0.01 (the original
        // value) sits inside that deadband; this was never noticed because
        // manual control was separately dead until the control-mode fix.
        // TODO: field-tune — this is a guess at "clears the deadband,
        // still gentle," not a validated crawl speed.
        public static final double manualDutyCycle = 0.08;

        public static final double[] intakePID = { 0.3, 0, 0 };
        public static final double[] intakeSVA = { 0, 0.13, 0.01 };

        // Jam-clearing: starting from intakingPosition (assumed already open when
        // AGITATE is pressed), the extension jogs back and forth while the rollers
        // keep spinning normally. Each close moves in by agitateCloseAmplitude, each
        // reopen only recovers agitateReopenAmplitude (less than the close amount),
        // so the whole oscillation band drifts toward closed over time instead of
        // just buzzing in place. First field test at period=0.25/close=3/reopen=2.5
        // came back "too big and too slow" -- reopen was too close to close (a slow
        // walk-down with big swings), not a fast buzz. Shrunk the swing size and
        // period, and shrunk reopen much further below close (not just slightly)
        // so it still drives to fully closed quickly despite the smaller steps:
        // 10 -> 9 -> 9.25 -> 8.25 -> 8.5 -> ... reaching closed in ~1.6s, then
        // settling into a small steady buzz right at the clamp (0 to
        // agitateReopenAmplitude) for as long as the button stays held.
        // TODO: field-tune all three -- still just a guess, not validated.
        public static final double agitatePeriodSeconds = 0.06;
        public static final double agitateCloseAmplitude = 1.0;
        public static final double agitateReopenAmplitude = 0.25;

        public enum IntakeWantedState {
            IDLE,
            INTAKE,
            RETRACT,
            RESET,
            SCORE,
            OUTTAKE,
            AGITATE,
            MANUAL_CONTROL_POS,
            MANUAL_CONTROL_NEG,
            MANUAL_IDLE,
            MANUAL_RESET
        }

        public enum SystemState {
            IDLING,
            INTAKING,
            RETRACTING,
            RESETING,
            SCORING,
            OUTTAKING,
            AGITATING,
            IN_MANUAL_CONTROL_POS,
            IN_MANUAL_CONTROL_NEG,
            IN_MANUAL_IDLE,
            IN_MANUAL_RESET
        }
    }

    public static class TurretConstants {
        public static final int SupplyCurrentLimit = 40;
        public static final int StatorCurrentLimit = 100;

        public static final int turretMotorID = 50;
        public static final int encoderID = 54;

        public static final double passAimPosition = 0;
        public static final double hubPresetPosition = 0;
        public static final double trenchPresetPositionL = .53;
        public static final double trenchPresetPositionR = .50;

        // Mechanism rotations. 0.05 rot = 18 degrees — generous on purpose
        // while SOTF is being tuned; tighten toward 0.007 once trusted.
        public static final double tolerance = 0.05;

        // Soft limits (mechanism rotations) and gearing.
        public static final double ccwLimit = 0.85;
        public static final double cwLimit = -0.85;
        public static final double gearRatio = 38.8888888889;

        // Manual debug control: how far the manual turret setpoint moves per
        // second of full joystick deflection. Unvalidated -- tune once on
        // real hardware, including checking the stick's sign convention.
        public static final double debugTurretRateRotPerSec = 0.5;

        public static final double[] turretPID = { 51, 0, 0 };
        public static final double[] turretSVA = { 0, 0, 0 };

        public enum TurretWantedState {
            IDLE,
            IDLE_AIM,
            AIM_PASS,
            AIM_HUB,
            TRENCH_PRESETL,
            TRENCH_PRESETR,
            HUB_PRESET,
            TEST,
            DEBUG
        }

        public enum SystemState {
            IDLING,
            IDLE_AIMING,
            PASS_AIMING,
            HUB_AIMING,
            TRENCH_PRESETTINGL,
            TRENCH_PRESETTINGR,
            HUB_PRESETTING,
            TESTING,
            DEBUGGING
        }
    }

    public static class FeederConstants {
        public static final int SupplyCurrentLimit = 40;
        public static final int StatorCurrentLimit = 100;

        public static final int towerMotorID = 41;
        public static final int spindexerMotorID = 40;
        public static final int rollerMotorID = 42;

        public static final double feederIntakeSpeed = 0;
        public static final double feederShootSpeed = 0.8;
        public static final double feederReverseSpeed = -0.7;

        // While PASSING, hold fire inside this field-Y band (the hub shadow)
        // so passes don't hit the hub structure.
        public static final double passExclusionYMin = 3.53;
        public static final double passExclusionYMax = 4.53;

        public enum FeederWantedState {
            IDLE,
            INTAKE,
            SHOOT,
            PASS,
            FEEDTEST
        }

        public enum SystemState {
            IDLING,
            INTAKING,
            SHOOTING,
            PASSING,
            FEEDTESTING
        }
    }

    public static class LightsConstants {
        // 60 LEDs per 1m strip [spacing = 1m / #ofLEDs]
        public static final Distance spacing = Meters.of(1.0 / 60.0);
        // LED Strip
        public static final int led_port = 0;
        public static final int led_length = 40; // 48 LEDs, 24 a side
        public static final int led_brightness = 30;
        // Signal LED sector (on shooter)
        public static final int signal_length = 10; // 10, 5 a side

        public enum LightsType {
            ENDGAME,
            CLIMB,
            SHOOTING,
            INTAKE,
            IDLE,
            DISABLED
        }

        // RGB Color Map
        public static final Map<String, Color> RGBColors = Map.ofEntries(
                Map.entry("black", new Color(0, 0, 0)),
                Map.entry("white", new Color(255, 255, 255)),
                Map.entry("red", new Color(255, 0, 0)),
                Map.entry("green", new Color(0, 255, 0)),
                Map.entry("blue", new Color(0, 0, 255)),
                Map.entry("gold", new Color(175, 184, 6)),
                Map.entry("team_Gold", new Color(179, 134, 27)),
                Map.entry("yellow", new Color(255, 255, 0)),
                Map.entry("orange", new Color(255, 165, 0)),
                Map.entry("pink", new Color(255, 20, 147)),
                Map.entry("magenta", new Color(255, 0, 255)),
                Map.entry("bright", new Color(234, 255, 48)));

        // RBG Color Map (new LEDs)
        public static final Map<String, Color> RBGColors = Map.ofEntries(
                Map.entry("black", new Color(0, 0, 0)),
                Map.entry("white", new Color(255, 255, 255)),
                Map.entry("red", new Color(255, 0, 0)),
                Map.entry("green", new Color(0, 0, 255)),
                Map.entry("blue", new Color(0, 255, 0)),
                Map.entry("gold", new Color(175, 6, 184)),
                Map.entry("team_Gold", new Color(179, 27, 134)),
                Map.entry("yellow", new Color(255, 0, 255)),
                Map.entry("orange", new Color(255, 0, 165)),
                Map.entry("pink", new Color(255, 147, 20)),
                Map.entry("magenta", new Color(255, 255, 0)),
                Map.entry("bright", new Color(234, 48, 255)));

        // GRB Color Map (old LEDs)
        public static final Map<String, Color> GRBColors = Map.ofEntries(
                Map.entry("black", new Color(0, 0, 0)),
                Map.entry("white", new Color(255, 255, 255)),
                Map.entry("red", new Color(0, 255, 0)),
                Map.entry("green", new Color(255, 0, 0)),
                Map.entry("blue", new Color(0, 0, 255)),
                Map.entry("gold", new Color(184, 175, 6)),
                Map.entry("team_Gold", new Color(134, 179, 27)),
                Map.entry("yellow", new Color(255, 255, 0)),
                Map.entry("orange", new Color(165, 255, 0)),
                Map.entry("pink", new Color(20, 255, 147)),
                Map.entry("magenta", new Color(0, 255, 255)),
                Map.entry("bright", new Color(255, 234, 48)));
    }

    public static class VisionConstants {
        public static final Transform2d turretToCenter = new Transform2d(
                Units.inchesToMeters(-6.5),
                Units.inchesToMeters(-6),
                new Rotation2d());

        public static final Transform3d kRobotToCam = new Transform3d(
                new Translation3d(
                        Units.inchesToMeters(6.643),
                        -Units.inchesToMeters(0.616),
                        Units.inchesToMeters(17.467 + 2.7525)),
                new Rotation3d(
                        0,
                        Units.degreesToRadians(20),
                        0));
        // center back cam
        public static final Transform3d kRobotToCam2 = new Transform3d(
                new Translation3d(
                        -Units.inchesToMeters(12.844),
                        Units.inchesToMeters(0.848),
                        Units.inchesToMeters(12.195)),
                new Rotation3d(
                        0,
                        Units.degreesToRadians(20),
                        Units.degreesToRadians(135)));
        // corner camera
        public static final Transform3d kRobotToCam3 = new Transform3d(
                new Translation3d(
                        -Units.inchesToMeters(12.843),
                        -Units.inchesToMeters(12.851),
                        Units.inchesToMeters(12.195)),
                new Rotation3d(
                        0,
                        Units.degreesToRadians(20),
                        Units.degreesToRadians(225)));

        public static final String cameraName = "camera1";
        public static final String camera2Name = "camera2";
        public static final String camera3Name = "camera3";

        /* standard deviations for vision calculations */
        public static final edu.wpi.first.math.Vector<N3> kSingleTagStdDevs = VecBuilder.fill(4, 4, 4);
        public static final edu.wpi.first.math.Vector<N3> kMultiTagStdDevs = VecBuilder.fill(1, 1, 4);
        public static final edu.wpi.first.math.Vector<N3> odoStdDEvs = VecBuilder.fill(.2, .2, .05);
        public static final double odometryUpdateFrequency = 250;
    }

    public static class FieldConstants {
        public enum ScoringZone {
            RED_PASSING_1,
            RED_PASSING_2,
            BLUE_PASSING_1,
            BLUE_PASSING_2,
            RED_HUB,
            BLUE_HUB,
            NO_TRACK
        }

        public static final Translation2d BLUE_HUB_POSE = new Translation2d(4.62, 4.03);
        public static final Translation2d RED_HUB_POSE = new Translation2d(12, 4.03);

        public static final Translation2d BLUE_PASS_SPOT_1 = new Translation2d(1, 3);
        public static final Translation2d BLUE_PASS_SPOT_2 = new Translation2d(1, 5);
        public static final Translation2d RED_PASS_SPOT_1 = new Translation2d(14.5, 3);
        public static final Translation2d RED_PASS_SPOT_2 = new Translation2d(14.5, 5);

        public static final Map<ScoringZone, Pose2d> scoringZoneLUT = Map.ofEntries(
                Map.entry(ScoringZone.RED_PASSING_1, new Pose2d(RED_PASS_SPOT_1, new Rotation2d())),
                Map.entry(ScoringZone.RED_PASSING_2, new Pose2d(RED_PASS_SPOT_2, new Rotation2d())),
                Map.entry(ScoringZone.RED_HUB, new Pose2d(RED_HUB_POSE, new Rotation2d())),
                Map.entry(ScoringZone.BLUE_PASSING_1, new Pose2d(BLUE_PASS_SPOT_1, new Rotation2d())),
                Map.entry(ScoringZone.BLUE_PASSING_2, new Pose2d(BLUE_PASS_SPOT_2, new Rotation2d())),
                Map.entry(ScoringZone.BLUE_HUB, new Pose2d(BLUE_HUB_POSE, new Rotation2d())),
                Map.entry(ScoringZone.NO_TRACK, Pose2d.kZero));
    }
}
