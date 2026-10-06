package frc.robot.subsystems.Intake;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.hardware.CANrange;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.IntakeConstants.IntakeWantedState;
import frc.robot.Constants.IntakeConstants.SystemState;

/**
 * Friction-wheel intake extension. The wheel is allowed to slip so the intake
 * can collapse when hit, which means the extension motor's encoder no longer
 * tracks where the intake actually is. So nothing here commands a position:
 * the extension is pushed at a fixed duty cycle toward a hard stop until it
 * stalls (or a timeout expires), then released to 0 output in Brake. After that the
 * wheel's grip is what holds it in place -- if another robot knocks it in, it
 * stays in until the driver asks for it again.
 */
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

    // Open-loop duty cycle, same as the rollers and the feeder motors.
    private double extensionDuty = 0.0;
    private double motorspeed = 0.0; // intake roller duty cycle

    private final DutyCycleOut extensionRequest = new DutyCycleOut(0.0);
    private final DutyCycleOut rollerRequest = new DutyCycleOut(0.0);

    // Push-to-hard-stop tracking. Restarted on every setWantedIntakeState() call
    // (not just on state changes) so re-requesting INTAKE while already INTAKING
    // re-extends an intake that got knocked in.
    private double pushStartTime = 0.0;
    private boolean pushFinished = false;
    private String pushEndReason = "none";

    // sim state -- a crude model so the stall logic can be exercised in sim:
    // extension moves proportional to duty cycle and stalls at 0 / simTravel.
    private double simExtensionPosition = 0.0;
    private double simExtensionVelocity = 0.0;
    private double simCanRangeDistance = 999.0;
    private static final double simTravel = 10.0;
    private static final double simRotPerSecAtFullOutput = 36.0;

    /* STATES */
    private IntakeWantedState wantedState = IntakeWantedState.IDLE;
    private SystemState systemState = SystemState.IDLING;

    // AGITATING toggle state -- persistent fields updated from
    // Timer.getFPGATimestamp(), not a Timer recreated inside applyState().
    // (An earlier oscillation attempt in git history did exactly that and never
    // actually toggled -- see commit dc7f908.) agitatePushingIn tracks which
    // pulse is currently running.
    private double agitateToggleTime = 0.0;
    private boolean agitatePushingIn = true;

    public Intake() {
        /* Extension motor: open loop, current limits cap push force */
        intakeExtensionMotorConfig.CurrentLimits.SupplyCurrentLimit = IntakeConstants.ExtensionSupplyCurrentLimit;
        intakeExtensionMotorConfig.CurrentLimits.StatorCurrentLimit = IntakeConstants.ExtensionStatorCurrentLimit;
        // Brake at 0 output so the extension resists creeping on its own; a real hit
        // still overcomes it (the friction wheel slips or the motor backdrives).
        intakeExtensionMotorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;

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

    // Sim safe helpers. Extension position is motor rotations since boot --
    // informational only, it drifts every time the friction wheel slips.
    private double getExtensionPosition() {
        if (Robot.isSimulation()) {
            return simExtensionPosition;
        }
        return intakeExtensionMotor.getPosition().getValueAsDouble();
    }

    private double getExtensionVelocity() {
        if (Robot.isSimulation()) {
            return simExtensionVelocity;
        }
        return intakeExtensionMotor.getVelocity().getValueAsDouble();
    }

    private double getExtensionStatorCurrent() {
        if (Robot.isSimulation()) {
            // Stalled against a sim end stop while pushing -> report high current.
            boolean atStop = (extensionDuty > 0 && simExtensionPosition >= simTravel)
                    || (extensionDuty < 0 && simExtensionPosition <= 0);
            return atStop ? IntakeConstants.ExtensionStatorCurrentLimit : 0.0;
        }
        return Math.abs(intakeExtensionMotor.getStatorCurrent().getValueAsDouble());
    }

    private double getCanRangeDistance() {
        if (Robot.isSimulation()) {
            return simCanRangeDistance;
        }
        return canRange.getDistance().getValueAsDouble();
    }

    /**
     * Also restarts the extension push, even if the state doesn't change --
     * that's what lets a second INTAKE press re-extend a collapsed intake.
     */
    public void setWantedIntakeState(IntakeWantedState desiredState) {
        this.wantedState = desiredState;
        pushStartTime = Timer.getFPGATimestamp();
        pushFinished = false;
        pushEndReason = "running";
    }

    private SystemState changeCurrentSystemState() {
        return switch (wantedState) {
            case IDLE -> SystemState.IDLING;
            case INTAKE -> SystemState.INTAKING;
            case RETRACT -> SystemState.RETRACTING;
            case SCORE -> SystemState.SCORING;
            case OUTTAKE -> SystemState.OUTTAKING;
            case AGITATE -> SystemState.AGITATING;
            case MANUAL_CONTROL_POS -> SystemState.IN_MANUAL_CONTROL_POS;
            case MANUAL_CONTROL_NEG -> SystemState.IN_MANUAL_CONTROL_NEG;
            case MANUAL_IDLE -> SystemState.IN_MANUAL_IDLE;
        };
    }

    /**
     * Push the extension at `duty` toward a hard stop, then release to 0 once
     * it stalls there or `timeoutSeconds` runs out. The timeout is the backstop
     * for when the friction wheel slips at the stop instead of stalling the
     * motor -- in that case the motor keeps spinning and no stall ever shows up.
     */
    private void pushToHardStop(double duty, double timeoutSeconds) {
        if (pushFinished) {
            extensionDuty = 0.0;
            return;
        }
        double elapsed = Timer.getFPGATimestamp() - pushStartTime;
        // Ignore the first moments of the push: current spikes and velocity is
        // still ~0 while the mechanism breaks free, which would look like a stall.
        boolean stalled = elapsed > IntakeConstants.extensionStallIgnoreSeconds
                && Math.abs(getExtensionVelocity()) < IntakeConstants.extensionStallVelocity
                && getExtensionStatorCurrent() > IntakeConstants.extensionStallCurrent;
        if (stalled || elapsed > timeoutSeconds) {
            pushFinished = true;
            pushEndReason = stalled ? "stall" : "timeout";
            extensionDuty = 0.0;
        } else {
            extensionDuty = duty;
        }
    }

    private void applyState() {
        switch (systemState) {
            case IDLING:
                motorspeed = 0.0;
                extensionDuty = 0.0;
                break;
            case INTAKING:
                motorspeed = IntakeConstants.intakingSpeed;
                pushToHardStop(IntakeConstants.extendDutyCycle, IntakeConstants.extensionPushTimeoutSeconds);
                break;
            case RETRACTING:
                pushToHardStop(-IntakeConstants.extendDutyCycle, IntakeConstants.extensionPushTimeoutSeconds);
                break;
            case SCORING:
                // Slow squeeze: same push-to-stop, just gentler and given longer to get there.
                pushToHardStop(-IntakeConstants.squeezeDutyCycle, IntakeConstants.squeezeTimeoutSeconds);
                break;
            case OUTTAKING:
                motorspeed = -IntakeConstants.intakingSpeed;
                extensionDuty = 0.0;
                break;
            case AGITATING:
                // Jam-clearing: rollers keep spinning while the extension is pulsed
                // in and out. In-pulses run longer than out-pulses, so the intake
                // walks toward closed over time rather than buzzing in place, then
                // keeps buzzing against the closed stop while the button is held.
                double pulseLength = agitatePushingIn
                        ? IntakeConstants.agitateInPulseSeconds
                        : IntakeConstants.agitateOutPulseSeconds;
                if (Timer.getFPGATimestamp() - agitateToggleTime > pulseLength) {
                    agitatePushingIn = !agitatePushingIn;
                    agitateToggleTime = Timer.getFPGATimestamp();
                }
                extensionDuty = agitatePushingIn ? -IntakeConstants.agitateDutyCycle : IntakeConstants.agitateDutyCycle;
                motorspeed = IntakeConstants.intakingSpeed;
                break;
            case IN_MANUAL_CONTROL_POS:
                extensionDuty = IntakeConstants.manualDutyCycle;
                break;
            case IN_MANUAL_CONTROL_NEG:
                extensionDuty = -IntakeConstants.manualDutyCycle;
                break;
            case IN_MANUAL_IDLE:
                extensionDuty = 0.0;
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

    public SystemState getState() {
        return systemState;
    }

    private void logValues() {
        SmartDashboard.putNumber("INTAKE/Extension Motor Position", getExtensionPosition());
        SmartDashboard.putNumber("INTAKE/Extension Velocity", getExtensionVelocity());
        SmartDashboard.putNumber("INTAKE/Extension Stator Current", getExtensionStatorCurrent());
        SmartDashboard.putNumber("INTAKE/Extension Duty Cycle", extensionDuty);
        // How the last push ended: "stall" is the healthy case; "timeout" every
        // time means the wheel is slipping at the stop (or thresholds are off).
        SmartDashboard.putString("INTAKE/Last Push End", pushEndReason);
        SmartDashboard.putNumber("INTAKE/CANrange Distance", getCanRangeDistance());
        SmartDashboard.putString("STATE/INTAKE WANTED STATE", wantedState.toString());
        SmartDashboard.putString("STATE/INTAKE SYSTEM STATE", systemState.toString());
    }

    @Override
    public void periodic() {
        logValues();

        SystemState nextState = changeCurrentSystemState();
        // Restart the pulse cycle fresh on every new AGITATING entry.
        if (nextState == SystemState.AGITATING && systemState != SystemState.AGITATING) {
            agitateToggleTime = Timer.getFPGATimestamp();
            agitatePushingIn = true;
        }
        systemState = nextState;

        applyState();

        if (Robot.isSimulation()) {
            double next = MathUtil.clamp(
                    simExtensionPosition + extensionDuty * simRotPerSecAtFullOutput * 0.02, 0.0, simTravel);
            simExtensionVelocity = (next - simExtensionPosition) / 0.02;
            simExtensionPosition = next;
        } else {
            intakeExtensionMotor.setControl(extensionRequest.withOutput(extensionDuty));
            intakeMotor.setControl(rollerRequest.withOutput(motorspeed));
            intakeMotor2.setControl(rollerRequest.withOutput(-motorspeed));
        }
    }
}
