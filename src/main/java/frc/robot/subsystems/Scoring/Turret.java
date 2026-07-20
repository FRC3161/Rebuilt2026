package frc.robot.subsystems.Scoring;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.Constants.TurretConstants;
import frc.robot.Constants.TurretConstants.SystemState;
import frc.robot.Constants.TurretConstants.TurretWantedState;
import frc.robot.subsystems.Drive.CommandSwerveDrivetrain;
import frc.util.Interpolation.LoggedTunableNumber;

public class Turret extends SubsystemBase {
    private final CommandSwerveDrivetrain drivetrain;

    /* MOTORS */
    private final TalonFX turretMotor = new TalonFX(TurretConstants.turretMotorID, CANBus.roboRIO());
    private final TalonFXConfiguration turretMotorConfig = new TalonFXConfiguration();

    /* ENCODERS */
    private final CANcoder encoder = new CANcoder(TurretConstants.encoderID, CANBus.roboRIO());

    // for position control (mechanism rotations)
    private double position = 0.0;
    private double simTurretPosition = 0.0;
    // Driver aim trim, in mechanism rotations. Applied before the soft-limit
    // normalization so trimming can never push the target past a limit.
    private double offset = 0.0;

    private final PositionVoltage positionRequest = new PositionVoltage(0);

    /* PIDFF CONTROL */
    private final LoggedTunableNumber k_S = new LoggedTunableNumber("turret_s", TurretConstants.turretSVA[0]);
    private final LoggedTunableNumber k_V = new LoggedTunableNumber("turret_v", TurretConstants.turretSVA[1]);
    private final LoggedTunableNumber k_A = new LoggedTunableNumber("turret_a", TurretConstants.turretSVA[2]);
    private final LoggedTunableNumber k_P = new LoggedTunableNumber("turret_p", TurretConstants.turretPID[0]);
    private final LoggedTunableNumber k_I = new LoggedTunableNumber("turret_i", TurretConstants.turretPID[1]);
    private final LoggedTunableNumber k_D = new LoggedTunableNumber("turret_d", TurretConstants.turretPID[2]);

    /* STATES */
    private TurretWantedState wantedState = TurretWantedState.IDLE;
    private SystemState systemState = SystemState.IDLING;

    public Turret(CommandSwerveDrivetrain drivetrain) {
        this.drivetrain = drivetrain;

        turretMotorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        turretMotorConfig.CurrentLimits.SupplyCurrentLimit = TurretConstants.SupplyCurrentLimit;
        turretMotorConfig.CurrentLimits.StatorCurrentLimit = TurretConstants.StatorCurrentLimit;
        turretMotorConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;
        turretMotorConfig.Feedback.SensorToMechanismRatio = TurretConstants.gearRatio;
        turretMotorConfig.ClosedLoopGeneral.ContinuousWrap = false;

        applyTunableGains();

        if (!Robot.isSimulation()) {
            StatusCode status = StatusCode.StatusCodeNotInitialized;
            for (int i = 0; i < 5; ++i) {
                status = turretMotor.getConfigurator().apply(turretMotorConfig);
                if (status.isOK())
                    break;
            }
            if (!status.isOK()) {
                System.out.println("Could not apply turret configs, error code: " + status.toString());
            }
            // Seed the motor's mechanism position from the absolute encoder so
            // the turret knows where it is after a power cycle.
            turretMotor.setPosition(encoder.getAbsolutePosition().getValue());
        }
    }

    private void applyTunableGains() {
        turretMotorConfig.Slot0.kS = k_S.get();
        turretMotorConfig.Slot0.kV = k_V.get();
        turretMotorConfig.Slot0.kA = k_A.get();
        turretMotorConfig.Slot0.kP = k_P.get();
        turretMotorConfig.Slot0.kI = k_I.get();
        turretMotorConfig.Slot0.kD = k_D.get();
    }

    // Returns current turret position (mechanism rotations), sim safe
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

    /**
     * Converts the field-frame aim angle from the SOTF solver into a turret
     * position target (mechanism rotations), taking the shortest path that
     * stays inside the soft limits.
     */
    private double calculateAimTarget() {
        double currentPosition = getTurretPosition();
        Rotation2d robotAngle = drivetrain.getPose().getRotation();

        // Compensate for chassis rotation between when the shot command was
        // solved and now.
        double dt = Timer.getFPGATimestamp() - drivetrain.shotCommandTimestamp;
        double omegaRad = drivetrain.getState().Speeds.omegaRadiansPerSecond;
        Rotation2d rotationCorrection = Rotation2d.fromRadians(omegaRad * dt);

        Rotation2d fieldAimAngle = drivetrain.currentShotCommand.turretAngle().plus(rotationCorrection);

        // Turret faces backwards relative to the chassis, hence the 180.
        Rotation2d desiredTurretAngle = fieldAimAngle
                .minus(robotAngle)
                .plus(Rotation2d.fromDegrees(180));

        double desiredRotations = desiredTurretAngle.getDegrees() / 360.0 + offset;
        double delta = Math.IEEEremainder(desiredRotations - currentPosition, 1.0);
        double target = currentPosition + delta;

        // Unwrap into the allowed range, then clamp as a final guarantee.
        while (target > TurretConstants.ccwLimit)
            target -= 1.0;
        while (target < TurretConstants.cwLimit)
            target += 1.0;
        return MathUtil.clamp(target, TurretConstants.cwLimit, TurretConstants.ccwLimit);
    }

    private void applyState() {
        switch (systemState) {
            case IDLING:
                position = 0.0;
                break;
            case IDLE_AIMING:
                // Track the target without committing to a shot — same math,
                // shooter simply isn't spun up.
                position = calculateAimTarget();
                break;
            case PASS_AIMING:
            case HUB_AIMING:
                position = calculateAimTarget();
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
        if (Robot.isSimulation())
            return true;

        // Passing involves fast chassis rotation that the turret can't mechanically
        // track within the tight hub-shot tolerance, so widen the window while
        // spinning fast. Stays tight when slow/stationary for accurate hub shots.
        double dynamicTolerance = TurretConstants.tolerance;
        double omega = Math.abs(drivetrain.getState().Speeds.omegaRadiansPerSecond);
        if (omega > 0.5) {
            dynamicTolerance = TurretConstants.tolerance * 3;
        } else if (omega > 0.2) {
            dynamicTolerance = TurretConstants.tolerance * 2;
        }

        return Math.abs(getTurretPosition() - position) < dynamicTolerance;
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

    // Exposed so RobotContainer can add this to the startup-jingle Orchestra individually.
    public TalonFX getTurretMotor() {
        return turretMotor;
    }

    public void checkTunableValues() {
        if (!Robot.isSimulation()) {
            if (k_S.hasChanged() || k_V.hasChanged() || k_A.hasChanged()
                    || k_P.hasChanged() || k_I.hasChanged() || k_D.hasChanged()) {
                applyTunableGains();
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
        checkTunableValues();
        logValues();
        systemState = changeCurrentSystemState();
        applyState();

        if (Robot.isSimulation()) {
            // In simulation, turret instantly reaches setpoint
            simTurretPosition = position;
        } else {
            turretMotor.setControl(positionRequest.withPosition(position));
        }
    }
}
