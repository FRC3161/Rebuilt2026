package frc.robot.subsystems.Intake;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.MotionMagicExpoVoltage;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.hardware.CANrange;
import com.ctre.phoenix6.hardware.TalonFX;

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
    private TalonFX intakeMotor = new TalonFX(IntakeConstants.intakeMotorID, "rio");
    public TalonFX intakeExtensionMotor = new TalonFX(IntakeConstants.intakeExtensionMotorID, "rio");
    private TalonFXConfiguration intakeMotorConfig = new TalonFXConfiguration();
    private TalonFXConfiguration intakeExtensionMotorConfig = new TalonFXConfiguration();

    /* SENSOR */
    private CANrange canRange = new CANrange(IntakeConstants.canRangeID, CANBus.roboRIO());

    // for velocity control
    private double motorspeed = 0.0;
    final MotionMagicVelocityVoltage mm_request = new MotionMagicVelocityVoltage(0);
    // for position control
    private double position = 0.0;
    final MotionMagicExpoVoltage mmE_request = new MotionMagicExpoVoltage(0);
    final DutyCycleOut m_leftrequestOut = new DutyCycleOut(0.0);

    // sim state
    private double simExtensionPosition = 0.0;
    private double simCanRangeDistance = 999.0; // default far away, won't trigger homing

    private LoggedTunableNumber k_S = new LoggedTunableNumber("intake_s", IntakeConstants.intakeSVA[0]);
    private LoggedTunableNumber k_V = new LoggedTunableNumber("intake_v", IntakeConstants.intakeSVA[1]);
    private LoggedTunableNumber k_A = new LoggedTunableNumber("intake_a", IntakeConstants.intakeSVA[2]);
    private LoggedTunableNumber k_P = new LoggedTunableNumber("intake_p", IntakeConstants.intakePID[0]);
    private LoggedTunableNumber k_I = new LoggedTunableNumber("intake_i", IntakeConstants.intakePID[1]);
    private LoggedTunableNumber k_D = new LoggedTunableNumber("intake_d", IntakeConstants.intakePID[2]);

    /* STATES */
    IntakeWantedState wantedState = IntakeWantedState.IDLE;
    SystemState systemState = SystemState.IDLING;

    /** Creates a new Intake */
    public Intake() {
        /* SETUP CONFIG */
        intakeExtensionMotorConfig.Slot0.kS = k_S.get();
        intakeExtensionMotorConfig.Slot0.kV = k_V.get();
        intakeExtensionMotorConfig.Slot0.kA = k_A.get();
        intakeExtensionMotorConfig.Slot0.kP = k_P.get();
        intakeExtensionMotorConfig.Slot0.kI = k_I.get();
        intakeExtensionMotorConfig.Slot0.kD = k_D.get();

        intakeMotorConfig.CurrentLimits.SupplyCurrentLimit = IntakeConstants.SupplyCurrentLimit;
        intakeMotorConfig.CurrentLimits.StatorCurrentLimit = IntakeConstants.StatorCurrentLimit;

        intakeExtensionMotorConfig.CurrentLimits.SupplyCurrentLimit = IntakeConstants.ExtensionSupplyCurrentLimit;
        intakeExtensionMotorConfig.CurrentLimits.StatorCurrentLimit = IntakeConstants.ExtensionStatorCurrentLimit;

        // Motion MAgic constants
        intakeExtensionMotorConfig.MotionMagic.MotionMagicExpo_kA = IntakeConstants.intakeMotionMagicExpoK_A;
        intakeExtensionMotorConfig.MotionMagic.MotionMagicExpo_kV = IntakeConstants.intakeMotionMagicExpoK_V;

        if (!Robot.isSimulation()) {
            StatusCode status = StatusCode.StatusCodeNotInitialized;
            for (int i = 0; i < 5; ++i) {
                status = intakeMotor.getConfigurator().apply(intakeMotorConfig);
                if (status.isOK())
                    break;
            }
            if (!status.isOK()) {
                System.out.println("Could not apply intake configs, error code: " + status.toString());
            }
            for (int i = 0; i < 5; ++i) {
                status = intakeExtensionMotor.getConfigurator().apply(intakeExtensionMotorConfig);
                if (status.isOK())
                    break;
            }
            if (!status.isOK()) {
                System.out.println("Could not apply configs, error code: " + status.toString());
            }

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
                intakeMotorConfig.Slot0.kS = k_S.get();
                intakeMotorConfig.Slot0.kV = k_V.get();
                intakeMotorConfig.Slot0.kA = k_A.get();
                intakeMotorConfig.Slot0.kP = k_P.get();
                intakeMotorConfig.Slot0.kI = k_I.get();
                intakeMotorConfig.Slot0.kD = k_D.get();
                intakeExtensionMotor.getConfigurator().apply(intakeMotorConfig);
            }
        }
    }

    public void setWantedIntakeState(IntakeWantedState desiredState) {
        this.wantedState = desiredState;
    }

    private SystemState changeCurrentSystemState() {
        return switch (wantedState) {
            case IDLE -> {
                if (systemState == SystemState.SCORING && !Robot.isSimulation()) {
                    intakeMotorConfig.Voltage.PeakReverseVoltage = IntakeConstants.intakeMotionMagicExpoK_A;
                    intakeExtensionMotor.getConfigurator().apply(intakeExtensionMotorConfig);
                }
                yield SystemState.IDLING;
            }
            case INTAKE -> {
                if (systemState == SystemState.SCORING && !Robot.isSimulation()) {
                    intakeMotorConfig.Voltage.PeakReverseVoltage = IntakeConstants.intakeMotionMagicExpoK_A;
                    intakeExtensionMotor.getConfigurator().apply(intakeExtensionMotorConfig);
                }
                // if (systemState == SystemState.INTAKING) {
                // yield SystemState.IDLING;
                // } else {
                yield SystemState.INTAKING;
            }
            case RETRACT -> {
                if (systemState == SystemState.SCORING && !Robot.isSimulation()) {
                    intakeMotorConfig.Voltage.PeakReverseVoltage = IntakeConstants.intakeMotionMagicExpoK_A;
                    intakeExtensionMotor.getConfigurator().apply(intakeMotorConfig);
                }
                yield SystemState.RETRACTING;
            }
            case RESET -> {
                if (systemState == SystemState.SCORING && !Robot.isSimulation()) {
                    intakeMotorConfig.Voltage.PeakReverseVoltage = IntakeConstants.intakeMotionMagicExpoK_A;
                    intakeExtensionMotor.getConfigurator().apply(intakeMotorConfig);
                }
                yield SystemState.RESETING;
            }
            case SCORE -> SystemState.SCORING;
            case OUTTAKE -> {
                if (systemState == SystemState.SCORING && !Robot.isSimulation()) {
                    intakeMotorConfig.Voltage.PeakReverseVoltage = IntakeConstants.intakeMotionMagicExpoK_A;
                    intakeExtensionMotor.getConfigurator().apply(intakeMotorConfig);
                }
                yield SystemState.OUTTAKING;
            }
            case MANUAL_CONTROL_POS -> {
                yield SystemState.IN_MANUAL_CONTROL_POS;
            }

            case MANUAL_CONTROL_NEG -> {
                yield SystemState.IN_MANUAL_CONTROL_NEG;
            }

            case MANUAL_IDLE -> {
                yield SystemState.IN_MANUAL_IDLE;
            }

            case MANUAL_RESET -> {
                yield SystemState.IN_MANUAL_RESET;
            }
        };
    }

    private void applyState() {
        switch (systemState) {
            case IDLING:
                motorspeed = 0.0;
                break;
            case INTAKING:
                position = IntakeConstants.intakingPosition;
                motorspeed = IntakeConstants.intakingSpeed;
                break;
            case RETRACTING:
                position = IntakeConstants.retractingPos;
                break;
            case RESETING:
                position = getExtensionPosition();
                if (getCanRangeDistance() > IntakeConstants.intakeExtensionHomingThreshold) {
                    if (Robot.isSimulation()) {
                        simExtensionPosition = 0.0;
                    } else {
                        intakeExtensionMotor.set(0);
                        intakeExtensionMotor.setPosition(0);
                    }
                } else {
                    if (!Robot.isSimulation()) {
                        intakeExtensionMotor.set(-0.01);
                    }
                }
                break;
            case SCORING:
                if (intakeMotorConfig.MotionMagic.MotionMagicExpo_kA != IntakeConstants.slowerIntakeKa) {
                    intakeMotorConfig.MotionMagic.MotionMagicExpo_kA = IntakeConstants.slowerIntakeKa;
                    if (!Robot.isSimulation()) {
                        intakeExtensionMotor.getConfigurator().apply(intakeMotorConfig);
                    }
                }
                position = 0;
                break;
            case OUTTAKING:
                motorspeed = -IntakeConstants.intakingSpeed;
                break;
            case IN_MANUAL_CONTROL_POS:
                position = intakeExtensionMotor.getPosition().getValueAsDouble();
                intakeExtensionMotor.set(0.01);
                break;
            case IN_MANUAL_CONTROL_NEG:
                position = intakeExtensionMotor.getPosition().getValueAsDouble();
                intakeExtensionMotor.set(-0.01);
                break;
            case IN_MANUAL_IDLE:
                position = intakeExtensionMotor.getPosition().getValueAsDouble();
                intakeExtensionMotor.set(0);
                break;
            case IN_MANUAL_RESET:
                setZero();
                break;

        }
    }

    public void enableEcoModeIntake() {
        if (!Robot.isSimulation()) {
            intakeMotorConfig.CurrentLimits.StatorCurrentLimit = 50;
            intakeMotorConfig.CurrentLimits.SupplyCurrentLimit = 50;
            intakeExtensionMotorConfig.CurrentLimits.StatorCurrentLimit = 30;
            intakeExtensionMotorConfig.CurrentLimits.SupplyCurrentLimit = 30;
            intakeExtensionMotor.getConfigurator().apply(intakeExtensionMotorConfig);
            intakeMotor.getConfigurator().apply(intakeMotorConfig);
        }
    }

    public void disableEcoModeIntake() {
        if (!Robot.isSimulation()) {
            intakeMotorConfig.CurrentLimits.StatorCurrentLimit = IntakeConstants.StatorCurrentLimit;
            intakeMotorConfig.CurrentLimits.SupplyCurrentLimit = IntakeConstants.SupplyCurrentLimit;
            intakeExtensionMotorConfig.CurrentLimits.StatorCurrentLimit = IntakeConstants.ExtensionStatorCurrentLimit;
            intakeExtensionMotorConfig.CurrentLimits.SupplyCurrentLimit = IntakeConstants.ExtensionSupplyCurrentLimit;
            intakeExtensionMotor.getConfigurator().apply(intakeExtensionMotorConfig);
            intakeMotor.getConfigurator().apply(intakeMotorConfig);
        }
    }

    public void setZero() {
        intakeExtensionMotor.setPosition(0);
    }

    private void canRangeControlledRecalibration() {
        if (canRange.getDistance().getValueAsDouble() > IntakeConstants.intakeExtensionHomingThreshold
                && intakeExtensionMotor.getPosition().getValueAsDouble() != 0) {
            intakeExtensionMotor.setPosition(0);
        }
    }

    public SystemState getState() {
        return systemState;
    }

    private void LogValues() {
        SmartDashboard.putNumber("INTAKE/Extension Motor Position", getExtensionPosition());
        SmartDashboard.putNumber("INTAKE/CANrange Distance", getCanRangeDistance());
        SmartDashboard.putString("STATE/INTAKE WANTED STATE", wantedState.toString());
        SmartDashboard.putString("STATE/INTAKE SYSTEM STATE", systemState.toString());
    }

    @Override
    public void periodic() {
        // canRangeControlledRecalibration();
        LogValues();
        systemState = changeCurrentSystemState();
        applyState();

        if (Robot.isSimulation()) {
            // In simulation, extension instantly reaches setpoint
            simExtensionPosition = position;
        } else {
            intakeExtensionMotor.setControl(mmE_request.withPosition(position));
            intakeMotor.setControl(m_leftrequestOut.withOutput(motorspeed));
        }
    }
}
