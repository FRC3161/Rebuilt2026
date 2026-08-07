package frc.robot.subsystems.Intake;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.MotionMagicExpoVoltage;
import com.ctre.phoenix6.hardware.CANrange;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.IntakeConstants.IntakeWantedState;
import frc.robot.Constants.IntakeConstants.SystemState;
import frc.util.Interpolation.LoggedTunableNumber;

public class Intake extends SubsystemBase {
    /* MOTORS */
    private final TalonFX intakeMotor = new TalonFX(IntakeConstants.intakeMotorID, CANBus.roboRIO());
    private final TalonFX intakeMotor2 = new TalonFX(IntakeConstants.intakeMotor2ID, CANBus.roboRIO());
    private final TalonFX intakeExtensionMotor = new TalonFX(IntakeConstants.intakeExtensionMotorID, CANBus.roboRIO());
    private final TalonFXConfiguration intakeMotorConfig = new TalonFXConfiguration();
    private final TalonFXConfiguration intakeMotor2Config = new TalonFXConfiguration();
    private final TalonFXConfiguration intakeExtensionMotorConfig = new TalonFXConfiguration();

    /* SENSOR */
    private final CANrange canRange = new CANrange(IntakeConstants.canRangeID, CANBus.roboRIO());

    /*
     * The extension motor runs in one of two modes each loop:
     * - POSITION: Motion Magic to `position`
     * - DUTY_CYCLE: open-loop `extensionDuty` (manual nudging + homing crawl)
     * Exactly one control call is made per loop based on this mode, so manual
     * control is never silently overridden by a position request.
     */
    private enum ExtensionControlMode {
        POSITION,
        DUTY_CYCLE
    }

    private ExtensionControlMode extensionMode = ExtensionControlMode.POSITION;
    private double extensionDuty = 0.0;
    private double position = 0.0;
    private double motorspeed = 0.0; // intake roller duty cycle

    private final MotionMagicExpoVoltage extensionPositionRequest = new MotionMagicExpoVoltage(0);
    private final DutyCycleOut rollerRequest = new DutyCycleOut(0.0);

    // sim state
    private double simExtensionPosition = 0.0;
    private double simCanRangeDistance = 999.0; // default far away, won't trigger homing

    private final LoggedTunableNumber k_S = new LoggedTunableNumber("intake_s", IntakeConstants.intakeSVA[0]);
    private final LoggedTunableNumber k_V = new LoggedTunableNumber("intake_v", IntakeConstants.intakeSVA[1]);
    private final LoggedTunableNumber k_A = new LoggedTunableNumber("intake_a", IntakeConstants.intakeSVA[2]);
    private final LoggedTunableNumber k_P = new LoggedTunableNumber("intake_p", IntakeConstants.intakePID[0]);
    private final LoggedTunableNumber k_I = new LoggedTunableNumber("intake_i", IntakeConstants.intakePID[1]);
    private final LoggedTunableNumber k_D = new LoggedTunableNumber("intake_d", IntakeConstants.intakePID[2]);

    /* STATES */
    private IntakeWantedState wantedState = IntakeWantedState.IDLE;
    private SystemState systemState = SystemState.IDLING;

    // AGITATING toggle state -- must be persistent instance fields updated once per
    // loop from Timer.getFPGATimestamp(), not a Timer recreated inside applyState().
    // (An earlier oscillation attempt in git history did exactly that and never
    // actually toggled -- see commit dc7f908.) agitateTarget is the drifting
    // position command; agitateExtended tracks whether the *next* toggle should
    // close (true, i.e. currently at an open step) or reopen (false).
    private double agitateToggleTime = 0.0;
    private double agitateTarget = 0.0;
    private boolean agitateExtended = true;

    public Intake() {
        /* Extension motor: closed-loop gains + Motion Magic profile */
        applyTunableGains();
        intakeExtensionMotorConfig.MotionMagic.MotionMagicExpo_kA = IntakeConstants.intakeMotionMagicExpoK_A;
        intakeExtensionMotorConfig.MotionMagic.MotionMagicExpo_kV = IntakeConstants.intakeMotionMagicExpoK_V;
        intakeExtensionMotorConfig.CurrentLimits.SupplyCurrentLimit = IntakeConstants.ExtensionSupplyCurrentLimit;
        intakeExtensionMotorConfig.CurrentLimits.StatorCurrentLimit = IntakeConstants.ExtensionStatorCurrentLimit;

        /* Roller motors: open loop, current limits only */
        intakeMotorConfig.CurrentLimits.SupplyCurrentLimit = IntakeConstants.SupplyCurrentLimit;
        intakeMotorConfig.CurrentLimits.StatorCurrentLimit = IntakeConstants.StatorCurrentLimit;
        intakeMotor2Config.CurrentLimits.SupplyCurrentLimit = IntakeConstants.SupplyCurrentLimit;
        intakeMotor2Config.CurrentLimits.StatorCurrentLimit = IntakeConstants.StatorCurrentLimit;

        if (!Robot.isSimulation()) {
            applyConfigWithRetry(intakeMotor, intakeMotorConfig, "intake roller");
            applyConfigWithRetry(intakeMotor2, intakeMotor2Config, "intake roller 2");
            applyConfigWithRetry(intakeExtensionMotor, intakeExtensionMotorConfig, "intake extension");
        }
    }

    private void applyTunableGains() {
        intakeExtensionMotorConfig.Slot0.kS = k_S.get();
        intakeExtensionMotorConfig.Slot0.kV = k_V.get();
        intakeExtensionMotorConfig.Slot0.kA = k_A.get();
        intakeExtensionMotorConfig.Slot0.kP = k_P.get();
        intakeExtensionMotorConfig.Slot0.kI = k_I.get();
        intakeExtensionMotorConfig.Slot0.kD = k_D.get();
    }

    private static void applyConfigWithRetry(TalonFX motor, TalonFXConfiguration config, String name) {
        StatusCode status = StatusCode.StatusCodeNotInitialized;
        for (int i = 0; i < 5; ++i) {
            status = motor.getConfigurator().apply(config);
            if (status.isOK())
                break;
        }
        if (!status.isOK()) {
            System.out.println("Could not apply " + name + " configs, error code: " + status.toString());
        }
    }

    // Sim safe helpers
    private double getExtensionPosition() {
        if (Robot.isSimulation()) {
            return simExtensionPosition;
        }
        return intakeExtensionMotor.getPosition().getValueAsDouble();
    }

    private double getCanRangeDistance() {
        if (Robot.isSimulation()) {
            return simCanRangeDistance;
        }
        return canRange.getDistance().getValueAsDouble();
    }

    public void checkTunableValues() {
        if (!Robot.isSimulation()) {
            if (k_S.hasChanged() || k_V.hasChanged() || k_A.hasChanged()
                    || k_P.hasChanged() || k_I.hasChanged() || k_D.hasChanged()) {
                applyTunableGains();
                intakeExtensionMotor.getConfigurator().apply(intakeExtensionMotorConfig);
            }
        }
    }

    public void setWantedIntakeState(IntakeWantedState desiredState) {
        this.wantedState = desiredState;
    }

    private SystemState changeCurrentSystemState() {
        return switch (wantedState) {
            case IDLE -> SystemState.IDLING;
            case INTAKE -> SystemState.INTAKING;
            case RETRACT -> SystemState.RETRACTING;
            case RESET -> SystemState.RESETING;
            case SCORE -> SystemState.SCORING;
            case OUTTAKE -> SystemState.OUTTAKING;
            case AGITATE -> SystemState.AGITATING;
            case MANUAL_CONTROL_POS -> SystemState.IN_MANUAL_CONTROL_POS;
            case MANUAL_CONTROL_NEG -> SystemState.IN_MANUAL_CONTROL_NEG;
            case MANUAL_IDLE -> SystemState.IN_MANUAL_IDLE;
            case MANUAL_RESET -> SystemState.IN_MANUAL_RESET;
        };
    }

    /**
     * SCORING slows the extension's Motion Magic profile (slow squeeze);
     * entering/leaving the state swaps the profile once. Both writes go to
     * the EXTENSION config and motor.
     */
    private void setExtensionProfileSlow(boolean slow) {
        double wantedKa = slow ? IntakeConstants.slowerIntakeKa : IntakeConstants.intakeMotionMagicExpoK_A;
        if (intakeExtensionMotorConfig.MotionMagic.MotionMagicExpo_kA != wantedKa) {
            intakeExtensionMotorConfig.MotionMagic.MotionMagicExpo_kA = wantedKa;
            if (!Robot.isSimulation()) {
                intakeExtensionMotor.getConfigurator().apply(intakeExtensionMotorConfig);
            }
        }
    }

    private void applyState() {
        switch (systemState) {
            case IDLING:
                motorspeed = 0.0;
                extensionMode = ExtensionControlMode.POSITION;
                break;
            case INTAKING:
                position = IntakeConstants.intakingPosition;
                motorspeed = IntakeConstants.intakingSpeed;
                extensionMode = ExtensionControlMode.POSITION;
                break;
            case RETRACTING:
                position = IntakeConstants.retractingPos;
                extensionMode = ExtensionControlMode.POSITION;
                break;
            case RESETING:
                // CANrange-based auto-homing disabled — the sensor isn't
                // mechanically reliable right now. Re-enable once it's
                // fixed; until then this just holds position like IDLING.
                //
                // Crawl inward until the CANrange sees the intake at home,
                // then zero. Threshold of 0 disables auto-zeroing.
                // if (IntakeConstants.intakeExtensionHomingThreshold > 0
                //         && getCanRangeDistance() > IntakeConstants.intakeExtensionHomingThreshold) {
                //     setZero();
                //     position = 0;
                //     extensionMode = ExtensionControlMode.POSITION;
                // } else {
                //     extensionDuty = -IntakeConstants.manualDutyCycle;
                //     extensionMode = ExtensionControlMode.DUTY_CYCLE;
                // }
                extensionMode = ExtensionControlMode.POSITION;
                break;
            case SCORING:
                position = 0;
                extensionMode = ExtensionControlMode.POSITION;
                break;
            case OUTTAKING:
                motorspeed = -IntakeConstants.intakingSpeed;
                extensionMode = ExtensionControlMode.POSITION;
                break;
            case AGITATING:
                // Jam-clearing: rollers keep spinning normally while the extension
                // jogs back and forth, each reopen recovering less than the
                // previous close so the band drifts toward closed over time (see
                // the Constants comment for the target sequence). Persistent timer,
                // not a Timer recreated in this method -- see the fields' comment.
                if (Timer.getFPGATimestamp() - agitateToggleTime > IntakeConstants.agitatePeriodSeconds) {
                    double delta = agitateExtended
                            ? -IntakeConstants.agitateCloseAmplitude
                            : IntakeConstants.agitateReopenAmplitude;
                    agitateTarget = MathUtil.clamp(
                            agitateTarget + delta, IntakeConstants.retractingPos, IntakeConstants.intakingPosition);
                    agitateExtended = !agitateExtended;
                    agitateToggleTime = Timer.getFPGATimestamp();
                }
                position = agitateTarget;
                motorspeed = IntakeConstants.intakingSpeed;
                extensionMode = ExtensionControlMode.POSITION;
                break;
            case IN_MANUAL_CONTROL_POS:
                extensionDuty = IntakeConstants.manualDutyCycle;
                extensionMode = ExtensionControlMode.DUTY_CYCLE;
                break;
            case IN_MANUAL_CONTROL_NEG:
                extensionDuty = -IntakeConstants.manualDutyCycle;
                extensionMode = ExtensionControlMode.DUTY_CYCLE;
                break;
            case IN_MANUAL_IDLE:
                extensionDuty = 0.0;
                extensionMode = ExtensionControlMode.DUTY_CYCLE;
                break;
            case IN_MANUAL_RESET:
                setZero();
                position = 0;
                extensionMode = ExtensionControlMode.POSITION;
                break;
        }
    }

    public void enableEcoModeIntake() {
        if (!Robot.isSimulation()) {
            intakeMotorConfig.CurrentLimits.StatorCurrentLimit = 50;
            intakeMotorConfig.CurrentLimits.SupplyCurrentLimit = 50;
            intakeMotor2Config.CurrentLimits.StatorCurrentLimit = 50;
            intakeMotor2Config.CurrentLimits.SupplyCurrentLimit = 50;
            intakeExtensionMotorConfig.CurrentLimits.StatorCurrentLimit = 30;
            intakeExtensionMotorConfig.CurrentLimits.SupplyCurrentLimit = 30;
            intakeExtensionMotor.getConfigurator().apply(intakeExtensionMotorConfig);
            intakeMotor.getConfigurator().apply(intakeMotorConfig);
            intakeMotor2.getConfigurator().apply(intakeMotor2Config);
        }
    }

    public void disableEcoModeIntake() {
        if (!Robot.isSimulation()) {
            intakeMotorConfig.CurrentLimits.StatorCurrentLimit = IntakeConstants.StatorCurrentLimit;
            intakeMotorConfig.CurrentLimits.SupplyCurrentLimit = IntakeConstants.SupplyCurrentLimit;
            intakeMotor2Config.CurrentLimits.StatorCurrentLimit = IntakeConstants.StatorCurrentLimit;
            intakeMotor2Config.CurrentLimits.SupplyCurrentLimit = IntakeConstants.SupplyCurrentLimit;
            intakeExtensionMotorConfig.CurrentLimits.StatorCurrentLimit = IntakeConstants.ExtensionStatorCurrentLimit;
            intakeExtensionMotorConfig.CurrentLimits.SupplyCurrentLimit = IntakeConstants.ExtensionSupplyCurrentLimit;
            intakeExtensionMotor.getConfigurator().apply(intakeExtensionMotorConfig);
            intakeMotor.getConfigurator().apply(intakeMotorConfig);
            intakeMotor2.getConfigurator().apply(intakeMotor2Config);
        }
    }

    // Exposed so RobotContainer can add this to the startup-jingle Orchestra individually.
    // Deliberately no getter for intakeMotor/intakeMotor2 — the rollers are excluded.
    public TalonFX getExtensionMotor() {
        return intakeExtensionMotor;
    }

    public void setZero() {
        if (Robot.isSimulation()) {
            simExtensionPosition = 0.0;
        } else {
            intakeExtensionMotor.setPosition(0);
        }
    }

    /** Manual escape hatch matching the CANrange auto-recalibration target. */
    public void setOut() {
        if (Robot.isSimulation()) {
            simExtensionPosition = 10.2;
        } else {
            intakeExtensionMotor.setPosition(10.2);
        }
    }

    public SystemState getState() {
        return systemState;
    }

    private void logValues() {
        SmartDashboard.putNumber("INTAKE/Extension Motor Position", getExtensionPosition());
        SmartDashboard.putNumber("INTAKE/CANrange Distance", getCanRangeDistance());
        SmartDashboard.putString("STATE/INTAKE WANTED STATE", wantedState.toString());
        SmartDashboard.putString("STATE/INTAKE SYSTEM STATE", systemState.toString());
    }

    @Override
    public void periodic() {
        checkTunableValues();
        logValues();

        SystemState nextState = changeCurrentSystemState();
        // Swap the extension's Motion Magic profile on SCORING transitions.
        if (nextState == SystemState.SCORING && systemState != SystemState.SCORING) {
            setExtensionProfileSlow(true);
        } else if (nextState != SystemState.SCORING && systemState == SystemState.SCORING) {
            setExtensionProfileSlow(false);
        }
        // Restart the jog fresh on every new AGITATING entry -- assumes the intake
        // is already open (matches the driver binding, which only reaches AGITATE
        // from INTAKE), so the drift starts from intakingPosition rather than
        // carrying stale state from a previous agitation.
        if (nextState == SystemState.AGITATING && systemState != SystemState.AGITATING) {
            agitateToggleTime = Timer.getFPGATimestamp();
            agitateTarget = IntakeConstants.intakingPosition;
            agitateExtended = true;
        }
        systemState = nextState;

        applyState();

        if (Robot.isSimulation()) {
            // In simulation, extension instantly reaches setpoint
            if (extensionMode == ExtensionControlMode.POSITION) {
                simExtensionPosition = position;
            }
        } else {
            if (extensionMode == ExtensionControlMode.POSITION) {
                intakeExtensionMotor.setControl(extensionPositionRequest.withPosition(position));
            } else {
                intakeExtensionMotor.set(extensionDuty);
            }
            intakeMotor.setControl(rollerRequest.withOutput(motorspeed));
            intakeMotor2.setControl(rollerRequest.withOutput(-motorspeed));
        }
    }
}
