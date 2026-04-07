package frc.robot.subsystems.Scoring;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.Constants.TurretConstants;
import frc.robot.Constants.TurretConstants.TurretWantedState;
import frc.robot.Constants.FieldConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.Drive.CommandSwerveDrivetrain;
import frc.util.Interpolation.LoggedTunableNumber;
import frc.robot.Constants.TurretConstants.SystemState;
import edu.wpi.first.wpilibj.Timer;

public class Turret extends SubsystemBase {
    private final CommandSwerveDrivetrain drivetrain;

    /* MOTORS */
    private TalonFX turretMotor = new TalonFX(TurretConstants.turretMotorID, "rio");
    private TalonFXConfiguration turretMotorConfig = new TalonFXConfiguration();

    /* ENCODERS */
    private CANcoder encoder = new CANcoder(TurretConstants.encoderID, "rio");

    // for position control
    private double position = 0.0;
    private double simTurretPosition = 0.0; // tracks simulated turret position
    private double CCWlimit = 0.85;
    private double CWLimit = -0.85;
    private double gearRatio = 38.8888888889;
    private double offset = 0.00;

    final PositionVoltage mmE_request = new PositionVoltage(0);

    /* PIDFF CONTROL */
    private LoggedTunableNumber k_S = new LoggedTunableNumber("turret_s", TurretConstants.turretSVA[0]);
    private LoggedTunableNumber k_V = new LoggedTunableNumber("turret_v", TurretConstants.turretSVA[1]);
    private LoggedTunableNumber k_A = new LoggedTunableNumber("turret_a", TurretConstants.turretSVA[2]);

    private LoggedTunableNumber k_P = new LoggedTunableNumber("turret_p", TurretConstants.turretPID[0]);
    private LoggedTunableNumber k_I = new LoggedTunableNumber("turret_i", TurretConstants.turretPID[1]);
    private LoggedTunableNumber k_D = new LoggedTunableNumber("turret_d", TurretConstants.turretPID[2]);

    /* STATES */
    TurretWantedState wantedState = TurretWantedState.IDLE;
    SystemState systemState = SystemState.IDLING;

    /** Creates a new Turret */
    public Turret(CommandSwerveDrivetrain drivetrain) {
        this.drivetrain = drivetrain;

        /* SETUP CONFIG */
        turretMotorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        turretMotorConfig.CurrentLimits.SupplyCurrentLimit = TurretConstants.SupplyCurrentLimit;
        turretMotorConfig.CurrentLimits.StatorCurrentLimit = TurretConstants.StatorCurrentLimit;
        turretMotorConfig.Feedback.FeedbackRemoteSensorID = 50;
        turretMotorConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;
        turretMotorConfig.Feedback.SensorToMechanismRatio = gearRatio;
        turretMotorConfig.ClosedLoopGeneral.ContinuousWrap = false;

        turretMotorConfig.Slot0.kS = k_S.get();
        turretMotorConfig.Slot0.kV = k_V.get();
        turretMotorConfig.Slot0.kA = k_A.get();
        turretMotorConfig.Slot0.kP = k_P.get();
        turretMotorConfig.Slot0.kI = k_I.get();
        turretMotorConfig.Slot0.kD = k_D.get();

        turretMotorConfig.MotionMagic.MotionMagicAcceleration = TurretConstants.turretMotionMagicAccel;
        turretMotorConfig.MotionMagic.MotionMagicJerk = TurretConstants.turretMotionMagicJerk;
        turretMotorConfig.MotionMagic.MotionMagicExpo_kV = TurretConstants.turretMotionMagicExpoK_V;
        turretMotorConfig.MotionMagic.MotionMagicExpo_kA = TurretConstants.turretMotionMagicExpoK_A;

        if (!Robot.isSimulation()) {
            StatusCode status = StatusCode.StatusCodeNotInitialized;
            for (int i = 0; i < 5; ++i) {
                status = turretMotor.getConfigurator().apply(turretMotorConfig);
                if (status.isOK())
                    break;
            }
            if (!status.isOK()) {
                System.out.println("Could not apply configs, error code: " + status.toString());
            }
            turretMotor.setPosition(encoder.getAbsolutePosition().getValue());
        }
    }

    // Returns current turret position, sim safe
    private double getTurretPosition() {
        if (Robot.isSimulation()) {
            return simTurretPosition;
        }
        return turretMotor.getPosition().getValueAsDouble();
    }

    public void setWantedTurretState(TurretWantedState desiredState) {
        this.wantedState = desiredState;
    }

    public void applyRightOffset() {
        offset += 0.01;
    }

    public void applyLeftOffset() {
        offset -= 0.01;
    }

    private SystemState changeCurrentSystemState() {
        return switch (wantedState) {
            case IDLE -> SystemState.IDLING;
            case IDLE_AIM -> SystemState.IDLE_AIMING;
            case AIM_HUB -> SystemState.HUB_AIMING;
            case AIM_PASS -> SystemState.PASS_AIMING;
            case TRENCH_PRESETL -> SystemState.TRENCH_PRESETTINGL;
            case TRENCH_PRESETR -> SystemState.TRENCH_PRESETTINGR;
            case HUB_PRESET -> SystemState.HUB_PRESETTING;
            case TEST -> SystemState.TESTING;
        };
    }

    private void applyState() {
        switch (systemState) {
            case IDLING:
                position = 0.0;
                break;

            case IDLE_AIMING:
                // commented out pending update
                break;

            case PASS_AIMING:
                double target = 0;
                double currentTurretToRobotAngle = getTurretPosition();
                Rotation2d currentRobotAngle = drivetrain.getPose().getRotation();

                double dt = Timer.getFPGATimestamp() - drivetrain.shotCommandTimestamp;
                double omegaRad = drivetrain.getState().Speeds.omegaRadiansPerSecond;
                Rotation2d rotationCorrection = Rotation2d.fromRadians(omegaRad * dt);

                Rotation2d angleToHub = drivetrain.currentShotCommand.turretAngle()
                        .plus(rotationCorrection);

                Rotation2d desiredTurretAngle = angleToHub
                        .minus(currentRobotAngle)
                        .plus(Rotation2d.fromDegrees(180));

                double convertedTurretAngle = desiredTurretAngle.getDegrees() / 360;
                double delta = convertedTurretAngle - currentTurretToRobotAngle;
                delta = Math.IEEEremainder(delta, 1.0);
                target = currentTurretToRobotAngle + delta;

                while (target > CCWlimit)
                    target -= 1.0;
                while (target < CWLimit)
                    target += 1.0;

                target += offset;
                position = target;
                break;

            case HUB_AIMING:
                double target2 = 0;
                double currentTurretToRobotAngle2 = getTurretPosition();
                Rotation2d currentRobotAngle2 = drivetrain.getPose().getRotation();
                // Rotation2d angleToHub2 = drivetrain.getSOTFTurretAngle().getAngle();

                /* OPTION 1 */
                // Rotation2d angleToHub2 = drivetrain.SOTF_CALC().getAngle();

                /* OPTION 2 */
                // Rotation2d angleToHub2 = Rotation2d.fromDegrees(drivetrain.SOTFcalc()[0]);

                /* OPTION 3 */
                // ChassisSpeeds rawFieldSpeeds = ChassisSpeeds.fromRobotRelativeSpeeds(
                // drivetrain.getState().Speeds,
                // drivetrain.getPose().getRotation());
                // Rotation2d angleToHub2 = (ShotCalc.calculateSOTF(
                // drivetrain.getTurretPose().getTranslation(),
                // rawFieldSpeeds, drivetrain.getScoringLocation(),
                // ShooterConstants.latencyCompensation).turretAngle());

                /* OPTION 4 */
                // Rotation2d angleToHub2 = ShotCalc2.calculateSOTF(drivetrain).turretAngle();

                // calculate desired angle of turret relative to hub
                // double angleToHub2 = (Math.atan2(drivetrain.getYfromHub(),
                // drivetrain.getXfromHub()));

                // calculate desired angle of turret relative to robot
                // Rotation2d desiredTurretAngle2 = (angleToHub2)

                double dt2 = Timer.getFPGATimestamp() - drivetrain.shotCommandTimestamp;
                double omegaRad2 = drivetrain.getState().Speeds.omegaRadiansPerSecond;
                Rotation2d rotationCorrection2 = Rotation2d.fromRadians(omegaRad2 * dt2);

                Rotation2d angleToHub2 = drivetrain.currentShotCommand.turretAngle()
                        .plus(rotationCorrection2);

                Rotation2d desiredTurretAngle2 = angleToHub2
                        .minus(currentRobotAngle2)
                        .plus(Rotation2d.fromDegrees(180));

                double convertedTurretAngle2 = desiredTurretAngle2.getDegrees() / 360;
                double delta2 = convertedTurretAngle2 - currentTurretToRobotAngle2;
                delta2 = Math.IEEEremainder(delta2, 1.0);
                target2 = currentTurretToRobotAngle2 + delta2;

                while (target2 > CCWlimit)
                    target2 -= 1.0;
                while (target2 < CWLimit)
                    target2 += 1.0;

                target2 += offset;
                position = target2;
                break;

            case TRENCH_PRESETTINGL:
                position = TurretConstants.trenchPresetPositionL;
                break;
            case TRENCH_PRESETTINGR:
                position = TurretConstants.trenchPresetPositionR;
                break;
            case HUB_PRESETTING:
                position = TurretConstants.hubPresetPosition;
                break;
            case TESTING:
                position = .75;
                break;
        }
    }

    public boolean turretIsReady() {
        return Math.abs(getTurretPosition() - position) < TurretConstants.tolerance;
    }

    public void enableEcoModeTurret() {
        if (!Robot.isSimulation()) {
            turretMotorConfig.CurrentLimits.StatorCurrentLimit = 40;
            turretMotorConfig.CurrentLimits.SupplyCurrentLimit = 40;
            turretMotor.getConfigurator().apply(turretMotorConfig);
        }
    }

    public void disableEcoModeTurret() {
        if (!Robot.isSimulation()) {
            turretMotorConfig.CurrentLimits.StatorCurrentLimit = TurretConstants.StatorCurrentLimit;
            turretMotorConfig.CurrentLimits.SupplyCurrentLimit = TurretConstants.SupplyCurrentLimit;
            turretMotor.getConfigurator().apply(turretMotorConfig);
        }
    }

    public void checkTunableValues() {
        if (!Robot.isSimulation()) {
            if (k_S.hasChanged() || k_V.hasChanged() || k_A.hasChanged()
                    || k_P.hasChanged() || k_I.hasChanged() || k_D.hasChanged()) {
                turretMotorConfig.Slot0.kS = k_S.get();
                turretMotorConfig.Slot0.kV = k_V.get();
                turretMotorConfig.Slot0.kA = k_A.get();
                turretMotorConfig.Slot0.kP = k_P.get();
                turretMotorConfig.Slot0.kI = k_I.get();
                turretMotorConfig.Slot0.kD = k_D.get();
                turretMotor.getConfigurator().apply(turretMotorConfig);
            }
        }
    }

    public TurretWantedState getState() {
        return wantedState;
    }

    private void logValues() {
        SmartDashboard.putNumber("TURRET/Turret Position", getTurretPosition());
        SmartDashboard.putNumber("TURRET/Turret Wanted Position", position);
        SmartDashboard.putNumber("TURRET/Turret Offset", offset);
        SmartDashboard.putBoolean("TURRET/Turret Is Ready", turretIsReady());
        SmartDashboard.putString("STATE/TURRET WANTED STATE", wantedState.toString());
        SmartDashboard.putString("STATE/TURRET SYSTEM STATE", systemState.toString());
        SmartDashboard.putNumber("SOTF/Shot Command Angle", drivetrain.currentShotCommand.turretAngle().getDegrees());
        SmartDashboard.putNumber("SOTF/Shot Command RPS", drivetrain.currentShotCommand.RPS());
        SmartDashboard.putNumber("SOTF/Shot Command Hood", drivetrain.currentShotCommand.hoodAngle());
        SmartDashboard.putNumber("SOTF/Rotation Correction Deg", Math.toDegrees(
                drivetrain.getState().Speeds.omegaRadiansPerSecond *
                        (Timer.getFPGATimestamp() - drivetrain.shotCommandTimestamp)));
        SmartDashboard.putNumber("SOTF/Robot Angle Deg", drivetrain.getPose().getRotation().getDegrees());
        SmartDashboard.putNumber("SOTF/Turret Field X", drivetrain.getCurrentTurretPose().getX());
        SmartDashboard.putNumber("SOTF/Turret Field Y", drivetrain.getCurrentTurretPose().getY());

        if (!Robot.isSimulation()) {
            SmartDashboard.putNumber("TURRET/Turret Absolute Position",
                    encoder.getAbsolutePosition().getValueAsDouble());
            SmartDashboard.putNumber("TURRET/Turret Motor Position", turretMotor.getPosition().getValueAsDouble());
            SmartDashboard.putNumber("TURRET/Turret Encoder Position", encoder.getPosition().getValueAsDouble());
        }
    }

    @Override
    public void periodic() {
        logValues();
        systemState = changeCurrentSystemState();
        applyState();

        if (Robot.isSimulation()) {
            // In simulation, turret instantly reaches setpoint
            simTurretPosition = position;
        } else {
            turretMotor.setControl(mmE_request.withPosition(position)
                    .withFeedForward(-drivetrain.getState().Speeds.omegaRadiansPerSecond));
        }
    }
}