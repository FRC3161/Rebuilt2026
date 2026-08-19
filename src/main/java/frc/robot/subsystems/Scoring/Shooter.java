package frc.robot.subsystems.Scoring;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;

import java.io.IOException;
import java.nio.file.Files;
import java.nio.file.Path;
import java.nio.file.StandardOpenOption;
import java.time.LocalDateTime;
import java.util.ArrayList;
import java.util.List;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.Constants.ShooterConstants;
import frc.robot.Constants.ShooterConstants.ShooterWantedState;
import frc.robot.Constants.ShooterConstants.SystemState;
import frc.robot.subsystems.Drive.CommandSwerveDrivetrain;
import frc.util.Interpolation.LoggedTunableNumber;

public class Shooter extends SubsystemBase {
    private final CommandSwerveDrivetrain drivetrain;

    /* MOTORS */
    private final TalonFX hoodMotor = new TalonFX(ShooterConstants.hoodMotorID, CANBus.roboRIO());
    private final TalonFXConfiguration hoodMotorConfig = new TalonFXConfiguration();
    private final TalonFX shooterMotor1 = new TalonFX(ShooterConstants.shooterMotor1ID, CANBus.roboRIO());
    private final TalonFX shooterMotor2 = new TalonFX(ShooterConstants.shooterMotor2ID, CANBus.roboRIO());
    private final TalonFXConfiguration shooterMotor1Config = new TalonFXConfiguration();
    private final TalonFXConfiguration shooterMotor2Config = new TalonFXConfiguration();

    // Plain velocity control, not motion-profiled: a flywheel should hit
    // target speed as fast as the motor/battery allow, not follow a smoothed
    // trapezoidal trajectory meant for mechanisms that need protecting from
    // abrupt accel.
    private final VelocityVoltage velocityRequest = new VelocityVoltage(0);
    private double motorspeed = 0.0;
    // hood position control
    private final PositionVoltage hoodPositionRequest = new PositionVoltage(0);
    private double position = 0.0;

    // sim state
    private double simShooterVelocity = 0.0;
    private double simHoodPosition = 0.0;

    // Manual debug-mode setpoints, driven by nudgeManualSetpoints(). Persist
    // across DEBUG activations so the operator doesn't lose their place.
    private double manualHoodPosition = 0.0;
    private double manualRPS = 0.0;
    private double lastDebugLoopTimestamp = -1;

    // Session-only log of shot-table marks (see markCurrentAsShotTablePoint), for the dashboard readout.
    private final List<String> markedPointsLog = new ArrayList<>();
    private static final Path shotMarkLogPath = Path.of(Filesystem.getOperatingDirectory().getPath(), "shot_table_marks.txt");

    /* PIDFF CONTROL */
    private final LoggedTunableNumber k_S = new LoggedTunableNumber("shooter_s", ShooterConstants.shooterSVA[0]);
    private final LoggedTunableNumber k_V = new LoggedTunableNumber("shooter_v", ShooterConstants.shooterSVA[1]);
    private final LoggedTunableNumber k_A = new LoggedTunableNumber("shooter_a", ShooterConstants.shooterSVA[2]);
    private final LoggedTunableNumber k_P = new LoggedTunableNumber("shooter_p", ShooterConstants.shooterPID[0]);
    private final LoggedTunableNumber k_I = new LoggedTunableNumber("shooter_i", ShooterConstants.shooterPID[1]);
    private final LoggedTunableNumber k_D = new LoggedTunableNumber("shooter_d", ShooterConstants.shooterPID[2]);

    /* STATES */
    private ShooterWantedState wantedState = ShooterWantedState.IDLE;
    private SystemState systemState = SystemState.IDLING;

    public Shooter(CommandSwerveDrivetrain m_drivetrain) {
        this.drivetrain = m_drivetrain;

        // CURRENT LIMITS
        hoodMotorConfig.CurrentLimits.SupplyCurrentLimit = ShooterConstants.hoodSupplyCurrentLimit;
        hoodMotorConfig.CurrentLimits.StatorCurrentLimit = ShooterConstants.hoodStatorCurrentLimit;
        shooterMotor1Config.CurrentLimits.SupplyCurrentLimit = ShooterConstants.SupplyCurrentLimit;
        shooterMotor1Config.CurrentLimits.StatorCurrentLimit = ShooterConstants.StatorCurrentLimit;
        shooterMotor2Config.CurrentLimits.SupplyCurrentLimit = ShooterConstants.SupplyCurrentLimit;
        shooterMotor2Config.CurrentLimits.StatorCurrentLimit = ShooterConstants.StatorCurrentLimit;

        // PID CONSTANTS
        hoodMotorConfig.Slot0.kS = ShooterConstants.hoodSVA[0];
        hoodMotorConfig.Slot0.kV = ShooterConstants.hoodSVA[1];
        hoodMotorConfig.Slot0.kA = ShooterConstants.hoodSVA[2];
        hoodMotorConfig.Slot0.kP = ShooterConstants.hoodPID[0];
        hoodMotorConfig.Slot0.kI = ShooterConstants.hoodPID[1];
        hoodMotorConfig.Slot0.kD = ShooterConstants.hoodPID[2];

        applyTunableGains();

        shooterMotor1Config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        shooterMotor2Config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        hoodMotorConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

        // Never drive the flywheel backwards.
        shooterMotor1Config.Voltage.PeakReverseVoltage = 0;
        shooterMotor2Config.Voltage.PeakReverseVoltage = 0;

        hoodMotorConfig.MotionMagic.MotionMagicExpo_kV = ShooterConstants.shooterMotionMagicExpoK_V;
        hoodMotorConfig.MotionMagic.MotionMagicExpo_kA = ShooterConstants.shooterMotionMagicExpoK_A;

        if (!Robot.isSimulation()) {
            applyConfigWithRetry(hoodMotor, hoodMotorConfig, "hood");
            applyConfigWithRetry(shooterMotor1, shooterMotor1Config, "shooter1");
            applyConfigWithRetry(shooterMotor2, shooterMotor2Config, "shooter2");

            hoodMotor.setPosition(0);

            // Stator current defaults to a slow CAN frame rate (unlike
            // velocity, which is kept fresh for closed-loop control); bump it
            // so current telemetry can resolve short transients like a game
            // piece passing through the flywheel.
            shooterMotor1.getStatorCurrent().setUpdateFrequency(50);
            shooterMotor2.getStatorCurrent().setUpdateFrequency(50);
        }
    }

    private void applyTunableGains() {
        shooterMotor1Config.Slot0.kS = k_S.get();
        shooterMotor1Config.Slot0.kV = k_V.get();
        shooterMotor1Config.Slot0.kA = k_A.get();
        shooterMotor1Config.Slot0.kP = k_P.get();
        shooterMotor1Config.Slot0.kI = k_I.get();
        shooterMotor1Config.Slot0.kD = k_D.get();

        shooterMotor2Config.Slot0.kS = k_S.get();
        shooterMotor2Config.Slot0.kV = k_V.get();
        shooterMotor2Config.Slot0.kA = k_A.get();
        shooterMotor2Config.Slot0.kP = k_P.get();
        shooterMotor2Config.Slot0.kI = k_I.get();
        shooterMotor2Config.Slot0.kD = k_D.get();
    }

    private static void applyConfigWithRetry(TalonFX motor, TalonFXConfiguration config, String name) {
        StatusCode status = StatusCode.StatusCodeNotInitialized;
        for (int i = 0; i < 5; ++i) {
            status = motor.getConfigurator().apply(config);
            if (status.isOK())
                break;
        }
        if (!status.isOK()) {
            System.out.println("Could not apply " + name + " configs, error code: "
                    + status.toString() + " id: " + motor.getDeviceID());
        }
    }

    // Sim safe helpers
    private double getShooterVelocity() {
        if (Robot.isSimulation()) {
            return simShooterVelocity;
        }
        return shooterMotor1.getVelocity().getValueAsDouble();
    }

    private double getShooter2Velocity() {
        if (Robot.isSimulation()) {
            return simShooterVelocity;
        }
        return shooterMotor2.getVelocity().getValueAsDouble();
    }

    private double getHoodPosition() {
        if (Robot.isSimulation()) {
            return simHoodPosition;
        }
        return hoodMotor.getPosition().getValueAsDouble();
    }

    private double getHoodCurrent() {
        if (Robot.isSimulation()) {
            return 0.0; // never triggers homing in sim
        }
        return hoodMotor.getSupplyCurrent().getValueAsDouble();
    }

    private double getShooterCurrent() {
        if (Robot.isSimulation()) {
            return 0.0;
        }
        return shooterMotor1.getStatorCurrent().getValueAsDouble();
    }

    private double getShooter2Current() {
        if (Robot.isSimulation()) {
            return 0.0;
        }
        return shooterMotor2.getStatorCurrent().getValueAsDouble();
    }

    public void setWantedShooterState(ShooterWantedState desiredState) {
        this.wantedState = desiredState;
    }

    private SystemState changeCurrentSystemState() {
        return switch (wantedState) {
            case IDLE -> SystemState.IDLING;
            case WAIT -> SystemState.ACTIVE_WAITING;
            case TRENCH_SHOOT -> SystemState.TRENCH_SHOOTING;
            case PASS_SHOOT -> SystemState.PASS_SHOOTING;
            case HUB_SHOOT -> SystemState.HUB_SHOOTING;
            case HOME -> SystemState.HOMING;
            case TEST -> SystemState.TESTING;
            case RETRACT_AUTO -> SystemState.RETRACTING_AUTO;
            case TURN_ON_AUTO -> SystemState.TURNING_ON_AUTO;
            case DEBUG -> SystemState.DEBUGGING;
        };
    }

    private void applyState() {
        switch (systemState) {
            case IDLING:
                motorspeed = 0.0;
                position = 0.0;
                break;
            case ACTIVE_WAITING:
                motorspeed = ShooterConstants.activeWaitingSpeed;
                position = 0.0;
                break;
            case INACTIVE_WAITING:
                motorspeed = ShooterConstants.inactiveWaitingSpeed;
                position = 0.0;
                break;
            case TRENCH_SHOOTING:
                motorspeed = 50;
                position = 5.5;
                break;
            case HUB_SHOOTING:
                motorspeed = drivetrain.currentShotCommand.RPS();
                position = MathUtil.clamp(drivetrain.currentShotCommand.hoodAngle(),
                        ShooterConstants.hoodMinPosition, ShooterConstants.hoodMaxPosition);
                break;
            case PASS_SHOOTING:

                // Predict future pose
                double lookAheadSeconds = 0.2;

                ChassisSpeeds fieldRelativeSpeeds = ChassisSpeeds.fromRobotRelativeSpeeds(
                        drivetrain.getState().Speeds,
                        drivetrain.getPose().getRotation());
                Pose2d currentPose = drivetrain.getPose();

                double predictedX = currentPose.getX() + fieldRelativeSpeeds.vxMetersPerSecond * lookAheadSeconds;
                double predictedY = currentPose.getY() + fieldRelativeSpeeds.vyMetersPerSecond * lookAheadSeconds;

                // Location Gate
                boolean inZoneCurrent = (currentPose.getY() > 6.6 || currentPose.getY() < 1.25)
                        && ((currentPose.getX() > 3.6 && currentPose.getX() < 5.45)
                                || (currentPose.getX() > 11.05 && currentPose.getX() < 12.9));

                boolean inZonePredicted = (predictedY > 6.6 || predictedY < 1.25)
                        && ((predictedX > 3.6 && predictedX < 5.45)
                                || (predictedX > 11.05 && predictedX < 12.9));

                boolean restrictHood = inZonePredicted || inZoneCurrent;

                motorspeed = drivetrain.currentShotCommand.RPS();
                position = restrictHood ? 0.0
                        : MathUtil.clamp(drivetrain.currentShotCommand.hoodAngle(),
                                ShooterConstants.hoodMinPosition, ShooterConstants.hoodMaxPosition);
                break;
            case HOMING:
                position = -0.1;
                // homingThreshold of 0 disables auto-zeroing (would trigger
                // instantly and re-zero the hood wherever it happens to be).
                if (ShooterConstants.homingThreshold > 0
                        && getHoodCurrent() >= ShooterConstants.homingThreshold) {
                    if (!Robot.isSimulation()) {
                        hoodMotor.setPosition(0);
                    }
                    simHoodPosition = 0.0;
                    position = 0;
                    setWantedShooterState(ShooterWantedState.IDLE);
                }
                break;
            case TESTING:
                motorspeed = 50;
                position = 0;
                break;
            case RETRACTING_AUTO:
                position = 0;
                break;
            case TURNING_ON_AUTO:
                motorspeed = 50;
                break;
            case DEBUGGING:
                motorspeed = manualRPS;
                position = manualHoodPosition;
                break;
        }
    }

    /**
     * Nudges the manual DEBUG-mode hood/flywheel setpoints by joystick
     * deflection, scaled by elapsed time and the tunable debug rates, and
     * clamped to the same safe limits as every other hood/RPS setpoint. Only
     * meaningful while the wanted state is DEBUG -- call once per loop from
     * whoever is driving debug mode.
     */
    public void nudgeManualSetpoints(double hoodStickInput, double rpsStickInput) {
        double now = Timer.getFPGATimestamp();
        double dt = (lastDebugLoopTimestamp < 0) ? 0.02 : now - lastDebugLoopTimestamp;
        lastDebugLoopTimestamp = now;

        manualHoodPosition = MathUtil.clamp(
                manualHoodPosition + hoodStickInput * ShooterConstants.debugHoodRateRotPerSec * dt,
                ShooterConstants.hoodMinPosition, ShooterConstants.hoodMaxPosition);
        manualRPS = MathUtil.clamp(
                manualRPS + rpsStickInput * ShooterConstants.debugRPSRatePerSec * dt,
                ShooterConstants.MIN_RPS, ShooterConstants.MAX_RPS);
    }

    /** Resets the debug nudge loop timer so re-entering DEBUG after a gap doesn't apply a huge dt jump. */
    public void resetManualDebugTimer() {
        lastDebugLoopTimestamp = -1;
    }

    /**
     * Snaps the manual DEBUG-mode hood/RPS setpoints to whatever ShotCalc is
     * currently recommending, as a tuning starting point -- so tuning can
     * start from a real number and adjust from there instead of hunting for
     * it by hand every time.
     */
    public void setManualToShotCalc() {
        manualRPS = MathUtil.clamp(drivetrain.currentShotCommand.RPS(),
                ShooterConstants.MIN_RPS, ShooterConstants.MAX_RPS);
        manualHoodPosition = MathUtil.clamp(drivetrain.currentShotCommand.hoodAngle(),
                ShooterConstants.hoodMinPosition, ShooterConstants.hoodMaxPosition);
    }

    /**
     * Marks the current manual DEBUG-mode hood/RPS setpoints as a new point
     * in the live RPS_MAP/HOOD_MAP tables, keyed to the robot's current real
     * distance from the hub. The table mutation itself is in-memory only --
     * lost on the next reboot/redeploy -- but every mark is also appended to
     * a session dashboard list and to a plain-text file on the RIO
     * (shotMarkLogPath), formatted as ready-to-paste RPS_MAP/HOOD_MAP.put()
     * lines, so good values are easy to carry into Constants.java afterward
     * even across a power cycle.
     */
    public void markCurrentAsShotTablePoint() {
        double distance = drivetrain.getDistanceFromHub();
        ShooterConstants.RPS_MAP.put(distance, manualRPS);
        ShooterConstants.HOOD_MAP.put(distance, manualHoodPosition);

        String summary = String.format("dist=%.2fm rps=%.1f hood=%.2f", distance, manualRPS, manualHoodPosition);
        SmartDashboard.putString("DEBUG/Last Marked Shot Point", summary);

        markedPointsLog.add(String.format("#%d %s", markedPointsLog.size() + 1, summary));
        SmartDashboard.putStringArray("DEBUG/Marked Shot Points", markedPointsLog.toArray(new String[0]));

        appendMarkToLogFile(distance);
    }

    /** Appends one mark to shotMarkLogPath as a timestamped, ready-to-paste RPS_MAP/HOOD_MAP.put() snippet. */
    private void appendMarkToLogFile(double distance) {
        String entry = String.format(
                "// %s -- marked during debug tuning%nRPS_MAP.put(%.2f, %.1fd);%nHOOD_MAP.put(%.2f, %.2fd);%n%n",
                LocalDateTime.now(), distance, manualRPS, distance, manualHoodPosition);
        try {
            Files.writeString(shotMarkLogPath, entry, StandardOpenOption.CREATE, StandardOpenOption.APPEND);
        } catch (IOException e) {
            System.out.println("Could not write shot table mark log: " + e.getMessage());
        }
    }

    public void checkTunableValues() {
        if (!Robot.isSimulation()) {
            if (k_S.hasChanged() || k_V.hasChanged() || k_A.hasChanged()
                    || k_P.hasChanged() || k_I.hasChanged() || k_D.hasChanged()) {
                applyTunableGains();
                shooterMotor1.getConfigurator().apply(shooterMotor1Config);
                shooterMotor2.getConfigurator().apply(shooterMotor2Config);
            }
        }
    }

    /**
     * True when the flywheel is at its commanded speed AND that speed is a
     * real shot (spinning) — never reports ready on a stopped flywheel, so
     * the feeder can't push a ball into it.
     */
    /** Manual escape hatch for the current-triggered auto-homing above. */
    public void hoodReset() {
        if (Robot.isSimulation()) {
            simHoodPosition = 0.0;
        } else {
            hoodMotor.setPosition(0);
        }
    }

    // Exposed so RobotContainer can add these to the startup-jingle Orchestra individually.
    public TalonFX getHoodMotor() {
        return hoodMotor;
    }

    public TalonFX getShooterMotor1() {
        return shooterMotor1;
    }

    public TalonFX getShooterMotor2() {
        return shooterMotor2;
    }

    public boolean shooterIsReady() {
        // Passing doesn't need hub-shot precision, and holding out for it just
        // delays the pass — accept a wider RPS window for that state only.
        double tolerance = systemState == SystemState.PASS_SHOOTING
                ? ShooterConstants.readyToleranceRPS * 3
                : ShooterConstants.readyToleranceRPS;
        return motorspeed > 1.0
                && Math.abs(getShooterVelocity() - motorspeed) < tolerance;
    }

    public void enableEcoModeShooter() {
        if (!Robot.isSimulation()) {
            shooterMotor1Config.CurrentLimits.StatorCurrentLimit = 40;
            shooterMotor1Config.CurrentLimits.SupplyCurrentLimit = 40;
            shooterMotor1.getConfigurator().apply(shooterMotor1Config);
            shooterMotor2Config.CurrentLimits.StatorCurrentLimit = 40;
            shooterMotor2Config.CurrentLimits.SupplyCurrentLimit = 40;
            shooterMotor2.getConfigurator().apply(shooterMotor2Config);
        }
    }

    public void disableEcoModeShooter() {
        if (!Robot.isSimulation()) {
            shooterMotor1Config.CurrentLimits.StatorCurrentLimit = ShooterConstants.StatorCurrentLimit;
            shooterMotor1Config.CurrentLimits.SupplyCurrentLimit = ShooterConstants.SupplyCurrentLimit;
            shooterMotor1.getConfigurator().apply(shooterMotor1Config);
            shooterMotor2Config.CurrentLimits.StatorCurrentLimit = ShooterConstants.StatorCurrentLimit;
            shooterMotor2Config.CurrentLimits.SupplyCurrentLimit = ShooterConstants.SupplyCurrentLimit;
            shooterMotor2.getConfigurator().apply(shooterMotor2Config);
        }
    }

    private void logValues() {
        SmartDashboard.putNumber("SHOOTER/Shooter Actual Speed", getShooterVelocity());
        SmartDashboard.putNumber("SHOOTER/Shooter Actual Speed 2", getShooter2Velocity());
        SmartDashboard.putNumber("SHOOTER/Hood Actual Position", getHoodPosition());
        SmartDashboard.putNumber("SHOOTER/Shooter Wanted Speed", motorspeed);
        SmartDashboard.putNumber("SHOOTER/Hood Wanted Position", position);
        SmartDashboard.putBoolean("SHOOTER/Shooter Is Ready", shooterIsReady());
        SmartDashboard.putBoolean("DEBUG/Manual Control Active", systemState == SystemState.DEBUGGING);
        SmartDashboard.putString("STATE/SHOOTER WANTED STATE", wantedState.toString());
        SmartDashboard.putString("STATE/SHOOTER SYSTEM STATE", systemState.toString());

        if (!Robot.isSimulation()) {
            SmartDashboard.putNumber("SHOOTER/Hood Motor Current", getHoodCurrent());
            SmartDashboard.putNumber("SHOOTER/Shooter Motor Current", getShooterCurrent());
            SmartDashboard.putNumber("SHOOTER/Shooter Motor 2 Current", getShooter2Current());
        }
    }

    @Override
    public void periodic() {
        checkTunableValues();
        logValues();
        systemState = changeCurrentSystemState();
        applyState();

        if (Robot.isSimulation()) {
            // In simulation, shooter and hood instantly reach setpoint
            simShooterVelocity = motorspeed;
            simHoodPosition = position;
        } else {
            hoodMotor.setControl(hoodPositionRequest.withPosition(position));
            shooterMotor1.setControl(velocityRequest.withVelocity(motorspeed));
            shooterMotor2.setControl(velocityRequest.withVelocity(motorspeed));
        }
    }
}
