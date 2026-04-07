package frc.robot.subsystems.Scoring;

import java.util.function.Function;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.Constants.ShooterConstants;
import frc.robot.Constants.ShooterConstants.ShooterWantedState;
import frc.robot.Constants.ShooterConstants.SystemState;
import frc.robot.subsystems.Drive.CommandSwerveDrivetrain;
import frc.robot.subsystems.Scoring.ShotCalc.ShooterCommand;
import frc.util.Interpolation.LoggedTunableNumber;

public class Shooter extends SubsystemBase {
    private CommandSwerveDrivetrain drivetrain;

    /* MOTORS */
    private TalonFX hoodMotor = new TalonFX(ShooterConstants.hoodMotorID, CANBus.roboRIO());
    private TalonFXConfiguration hoodMotorConfig = new TalonFXConfiguration();
    private TalonFX shooterMotor1 = new TalonFX(ShooterConstants.shooterMotor1ID, CANBus.roboRIO());
    private TalonFX shooterMotor2 = new TalonFX(ShooterConstants.shooterMotor2ID, CANBus.roboRIO());
    private TalonFXConfiguration shooterMotor1Config = new TalonFXConfiguration();
    private TalonFXConfiguration shooterMotor2Config = new TalonFXConfiguration();

    // for velocity control
    final DutyCycleOut t_request = new DutyCycleOut(0);
    private double motorspeed = 0.0;
    final MotionMagicVelocityVoltage mm_request = new MotionMagicVelocityVoltage(0);
    // for position control
    private double position = 0.0;
    final PositionVoltage mmE_request = new PositionVoltage(0);

    // sim state
    private double simShooterVelocity = 0.0;
    private double simHoodPosition = 0.0;

    /* PIDFF CONTROL */
    private LoggedTunableNumber k_S = new LoggedTunableNumber("shooter_s", ShooterConstants.shooterSVA[0]);
    private LoggedTunableNumber k_V = new LoggedTunableNumber("shooter_v", ShooterConstants.shooterSVA[1]);
    private LoggedTunableNumber k_A = new LoggedTunableNumber("shooter_a", ShooterConstants.shooterSVA[2]);
    private LoggedTunableNumber k_P = new LoggedTunableNumber("shooter_p", ShooterConstants.shooterPID[0]);
    private LoggedTunableNumber k_I = new LoggedTunableNumber("shooter_i", ShooterConstants.shooterPID[1]);
    private LoggedTunableNumber k_D = new LoggedTunableNumber("shooter_d", ShooterConstants.shooterPID[2]);

    /* STATES */
    ShooterWantedState wantedState = ShooterWantedState.IDLE;
    SystemState systemState = SystemState.IDLING;
    static SystemState publicSystemState = SystemState.IDLING;

    /** Creates a new Shooter */
    public Shooter(CommandSwerveDrivetrain m_drivetrain) {
        this.drivetrain = m_drivetrain;

        // CURRENT LIMITS
        hoodMotorConfig.CurrentLimits.SupplyCurrentLimit = ShooterConstants.hoodSupplyCurrentLimit;
        hoodMotorConfig.CurrentLimits.StatorCurrentLimit = ShooterConstants.hoodStatorCurrentLimit;
        shooterMotor2Config.CurrentLimits.SupplyCurrentLimit = ShooterConstants.SupplyCurrentLimit;
        shooterMotor2Config.CurrentLimits.StatorCurrentLimit = ShooterConstants.StatorCurrentLimit;
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

        shooterMotor1Config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        shooterMotor2Config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        hoodMotorConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

        shooterMotor1Config.Voltage.PeakReverseVoltage = 0;
        shooterMotor2Config.Voltage.PeakReverseVoltage = 0;

        shooterMotor1Config.MotionMagic.MotionMagicAcceleration = ShooterConstants.shooterMotionMagicAccel;
        shooterMotor1Config.MotionMagic.MotionMagicJerk = ShooterConstants.shooterMotionMagicJerk;
        shooterMotor2Config.MotionMagic.MotionMagicAcceleration = ShooterConstants.shooterMotionMagicAccel;
        shooterMotor2Config.MotionMagic.MotionMagicJerk = ShooterConstants.shooterMotionMagicJerk;

        hoodMotorConfig.MotionMagic.MotionMagicExpo_kV = ShooterConstants.shooterMotionMagicExpoK_V;
        hoodMotorConfig.MotionMagic.MotionMagicExpo_kA = ShooterConstants.shooterMotionMagicExpoK_A;

        if (!Robot.isSimulation()) {
            StatusCode hoodMotorStatus = StatusCode.StatusCodeNotInitialized;
            for (int i = 0; i < 5; ++i) {
                hoodMotorStatus = hoodMotor.getConfigurator().apply(hoodMotorConfig);
                if (hoodMotorStatus.isOK())
                    break;
            }
            if (!hoodMotorStatus.isOK()) {
                System.out.println("Could not apply hood configs, error code: "
                        + hoodMotorStatus.toString() + hoodMotor.getDeviceID());
            }

            StatusCode shooterMotor1Status = StatusCode.StatusCodeNotInitialized;
            for (int i = 0; i < 5; ++i) {
                shooterMotor1Status = shooterMotor1.getConfigurator().apply(shooterMotor1Config);
                if (shooterMotor1Status.isOK())
                    break;
            }
            if (!shooterMotor1Status.isOK()) {
                System.out.println("Could not apply shooter1 configs, error code: "
                        + shooterMotor1Status.toString() + shooterMotor1.getDeviceID());
            }

            StatusCode shooterMotor2Status = StatusCode.StatusCodeNotInitialized;
            for (int i = 0; i < 5; ++i) {
                shooterMotor2Status = shooterMotor2.getConfigurator().apply(shooterMotor2Config);
                if (shooterMotor2Status.isOK())
                    break;
            }
            if (!shooterMotor2Status.isOK()) {
                System.out.println("Could not apply shooter2 configs, error code: "
                        + shooterMotor2Status.toString() + shooterMotor2.getDeviceID());
            }

            hoodMotor.setPosition(0);
        }
    }

    // Sim safe helpers
    private double getShooterVelocity() {
        if (Robot.isSimulation()) {
            return simShooterVelocity;
        }
        return shooterMotor1.getVelocity().getValueAsDouble();
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
                // Translation2d correctedVector = drivetrain.getSOTFTurretAngle();
                // option 1
                // Translation2d correctedVector = drivetrain.SOTF_CALC();
                // double correctedDistance = correctedVector.getNorm();

                // motorspeed = ShooterConstants.shooterSpeedInterpolation
                // .getPrediction(correctedDistance);

                // position = MathUtil.clamp(
                // ShooterConstants.hoodAngleInterpolation.getPrediction(correctedDistance),
                // -0.5,
                // 8);
                // option 2
                // position = drivetrain.SOTFcalc()[1] /
                // ShooterConstants.hoodConversionRotToDeg;
                // motorspeed = drivetrain.SOTFcalc()[2];

                // option 3
                // ChassisSpeeds rawFieldSpeeds = ChassisSpeeds.fromRobotRelativeSpeeds(
                // drivetrain.getState().Speeds,
                // drivetrain.getPose().getRotation());
                // position = ShotCalc.calculateSOTF(
                // drivetrain.getPose().getTranslation(),
                // rawFieldSpeeds,
                // drivetrain.getScoringLocation(),
                // ShooterConstants.latencyCompensation).hoodAngle();
                // motorspeed = ShotCalc.calculateSOTF(
                // drivetrain.getPose().getTranslation(),
                // rawFieldSpeeds,
                // drivetrain.getScoringLocation(),
                // ShooterConstants.latencyCompensation).RPS();
                // option 4
                // frc.robot.subsystems.Scoring.ShotCalc2.ShooterCommand values =
                // ShotCalc2.calculateSOTF(drivetrain);
                // position = values.hoodAngle();
                // motorspeed = values.RPS();
                motorspeed = drivetrain.currentShotCommand.RPS();
                position = MathUtil.clamp(drivetrain.currentShotCommand.hoodAngle(), -0.5, 8);
                break;
            case PASS_SHOOTING:
                motorspeed = drivetrain.currentShotCommand.RPS();
                position = MathUtil.clamp(drivetrain.currentShotCommand.hoodAngle(), -0.5, 8);
                break;
            case HOMING:
                position = -0.1;
                if (getHoodCurrent() >= ShooterConstants.homingThreshold) {
                    if (!Robot.isSimulation()) {
                        hoodMotor.setPosition(0);
                    }
                    simHoodPosition = 0.0;
                    position = 0;
                    setWantedShooterState(ShooterWantedState.IDLE);
                }
                break;
            case TESTING:
                motorspeed = 95;
                position = 7.5;
                break;
            case RETRACTING_AUTO:
                position = 0;
                break; // fix: was falling through to TURNING_ON_AUTO
            case TURNING_ON_AUTO:
                motorspeed = 50;
                break;
        }
    }

    public void checkTunableValues() {
        if (!Robot.isSimulation()) {
            if (k_S.hasChanged() || k_V.hasChanged() || k_A.hasChanged()
                    || k_P.hasChanged() || k_I.hasChanged() || k_D.hasChanged()) {
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

                shooterMotor1.getConfigurator().apply(shooterMotor1Config);
                shooterMotor2.getConfigurator().apply(shooterMotor2Config);
            }
        }
    }

    public boolean shooterIsReady() {
        return Math.abs(getShooterVelocity() - motorspeed) < 1.5;
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

    public static SystemState getState() {
        return publicSystemState;
    }

    private void logValues() {
        SmartDashboard.putNumber("SHOOTER/Shooter Actual Speed", getShooterVelocity());
        SmartDashboard.putNumber("SHOOTER/Hood Actual Position", getHoodPosition());
        SmartDashboard.putNumber("SHOOTER/Shooter Wanted Speed", motorspeed);
        SmartDashboard.putNumber("SHOOTER/Hood Wanted Position", position);
        SmartDashboard.putBoolean("SHOOTER/Shooter Is Ready", shooterIsReady());
        SmartDashboard.putString("STATE/SHOOTER WANTED STATE", wantedState.toString());
        SmartDashboard.putString("STATE/SHOOTER SYSTEM STATE", systemState.toString());

        if (!Robot.isSimulation()) {
            SmartDashboard.putNumber("Hood Motor Current", getHoodCurrent());
        }
    }

    @Override
    public void periodic() {
        logValues();
        systemState = changeCurrentSystemState();
        applyState();

        if (Robot.isSimulation()) {
            // In simulation, shooter and hood instantly reach setpoint
            simShooterVelocity = motorspeed;
            simHoodPosition = position;
        } else {
            hoodMotor.setControl(mmE_request.withPosition(position));
            shooterMotor1.setControl(mm_request.withVelocity(motorspeed));
            shooterMotor2.setControl(mm_request.withVelocity(motorspeed));
        }
    }
}