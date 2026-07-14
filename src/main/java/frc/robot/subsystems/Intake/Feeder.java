package frc.robot.subsystems.Intake;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.Constants.FeederConstants;
import frc.robot.Constants.FeederConstants.FeederWantedState;
import frc.robot.Constants.FeederConstants.SystemState;
import frc.robot.subsystems.Drive.CommandSwerveDrivetrain;
import frc.robot.subsystems.Scoring.Shooter;
import frc.robot.subsystems.Scoring.Turret;

public class Feeder extends SubsystemBase {
    private final Turret turret;
    private final Shooter shooter;
    private final CommandSwerveDrivetrain drivetrain;

    /* MOTORS */
    private final TalonFX spindexerMotor = new TalonFX(FeederConstants.spindexerMotorID, CANBus.roboRIO());
    private final TalonFXConfiguration spindexerMotorConfig = new TalonFXConfiguration();
    private final TalonFX towerMotor = new TalonFX(FeederConstants.towerMotorID, CANBus.roboRIO());
    private final TalonFXConfiguration towerMotorConfig = new TalonFXConfiguration();
    private final TalonFX rollerMotor = new TalonFX(FeederConstants.rollerMotorID, CANBus.roboRIO());
    private final TalonFXConfiguration rollerMotorConfig = new TalonFXConfiguration();

    private double spindexerMotorSpeed = 0.0;
    private double towerMotorSpeed = 0.0;
    private double rollerMotorSpeed = 0.0;

    /* STATES */
    private FeederWantedState wantedState = FeederWantedState.IDLE;
    private SystemState systemState = SystemState.IDLING;

    public Feeder(Turret m_turret, Shooter m_shooter, CommandSwerveDrivetrain m_Drivetrain) {
        this.turret = m_turret;
        this.shooter = m_shooter;
        this.drivetrain = m_Drivetrain;

        // CURRENT LIMITS
        spindexerMotorConfig.CurrentLimits.SupplyCurrentLimit = FeederConstants.SupplyCurrentLimit;
        spindexerMotorConfig.CurrentLimits.StatorCurrentLimit = FeederConstants.StatorCurrentLimit;
        towerMotorConfig.CurrentLimits.SupplyCurrentLimit = FeederConstants.SupplyCurrentLimit;
        towerMotorConfig.CurrentLimits.StatorCurrentLimit = FeederConstants.StatorCurrentLimit;
        rollerMotorConfig.CurrentLimits.SupplyCurrentLimit = FeederConstants.SupplyCurrentLimit;
        rollerMotorConfig.CurrentLimits.StatorCurrentLimit = FeederConstants.StatorCurrentLimit;

        if (!Robot.isSimulation()) {
            applyConfigWithRetry(spindexerMotor, spindexerMotorConfig, "spindexer");
            applyConfigWithRetry(towerMotor, towerMotorConfig, "tower");
            applyConfigWithRetry(rollerMotor, rollerMotorConfig, "roller");
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

    public void setWantedFeederState(FeederWantedState desiredState) {
        this.wantedState = desiredState;
    }

    private SystemState changeCurrentSystemState() {
        return switch (wantedState) {
            case IDLE -> SystemState.IDLING;
            case INTAKE -> SystemState.INTAKING;
            // Only feed once the flywheel is at speed and the turret is on
            // target — otherwise hold the ball.
            case SHOOT -> (shooter.shooterIsReady() && turret.turretIsReady())
                    ? SystemState.SHOOTING
                    : SystemState.IDLING;
            case PASS -> (shooter.shooterIsReady() && turret.turretIsReady())
                    ? SystemState.PASSING
                    : SystemState.IDLING;
            case FEEDTEST -> SystemState.FEEDTESTING;
        };
    }

    private void applyState() {
        switch (systemState) {
            case IDLING:
                spindexerMotorSpeed = 0.0;
                towerMotorSpeed = 0.0;
                rollerMotorSpeed = 0.0;
                break;
            case INTAKING:
                spindexerMotorSpeed = FeederConstants.feederIntakeSpeed;
                towerMotorSpeed = 0.0;
                rollerMotorSpeed = FeederConstants.feederIntakeSpeed;
                break;
            case SHOOTING:
                spindexerMotorSpeed = FeederConstants.feederShootSpeed;
                towerMotorSpeed = FeederConstants.feederShootSpeed;
                rollerMotorSpeed = 0.0;
                break;
            case PASSING:
                // Hold fire while inside the hub shadow so passes don't hit
                // the hub structure.
                double y = drivetrain.getPose().getY();
                boolean inExclusionZone = y > FeederConstants.passExclusionYMin
                        && y < FeederConstants.passExclusionYMax;
                if (inExclusionZone) {
                    spindexerMotorSpeed = 0;
                    towerMotorSpeed = 0;
                    rollerMotorSpeed = 0;
                } else {
                    spindexerMotorSpeed = FeederConstants.feederShootSpeed;
                    towerMotorSpeed = FeederConstants.feederShootSpeed;
                    rollerMotorSpeed = 0.0;
                }
                break;
            case FEEDTESTING:
                spindexerMotorSpeed = FeederConstants.feederReverseSpeed;
                towerMotorSpeed = FeederConstants.feederReverseSpeed;
                rollerMotorSpeed = 0.0;
                break;
        }
    }

    public void enableEcoModeFeeder() {
        if (!Robot.isSimulation()) {
            towerMotorConfig.CurrentLimits.StatorCurrentLimit = 40;
            towerMotorConfig.CurrentLimits.SupplyCurrentLimit = 40;
            towerMotor.getConfigurator().apply(towerMotorConfig);
            spindexerMotorConfig.CurrentLimits.StatorCurrentLimit = 40;
            spindexerMotorConfig.CurrentLimits.SupplyCurrentLimit = 40;
            spindexerMotor.getConfigurator().apply(spindexerMotorConfig);
            rollerMotorConfig.CurrentLimits.StatorCurrentLimit = 40;
            rollerMotorConfig.CurrentLimits.SupplyCurrentLimit = 40;
            rollerMotor.getConfigurator().apply(rollerMotorConfig);
        }
    }

    public void disableEcoModeFeeder() {
        if (!Robot.isSimulation()) {
            towerMotorConfig.CurrentLimits.StatorCurrentLimit = FeederConstants.StatorCurrentLimit;
            towerMotorConfig.CurrentLimits.SupplyCurrentLimit = FeederConstants.SupplyCurrentLimit;
            towerMotor.getConfigurator().apply(towerMotorConfig);
            spindexerMotorConfig.CurrentLimits.StatorCurrentLimit = FeederConstants.StatorCurrentLimit;
            spindexerMotorConfig.CurrentLimits.SupplyCurrentLimit = FeederConstants.SupplyCurrentLimit;
            spindexerMotor.getConfigurator().apply(spindexerMotorConfig);
            rollerMotorConfig.CurrentLimits.StatorCurrentLimit = FeederConstants.StatorCurrentLimit;
            rollerMotorConfig.CurrentLimits.SupplyCurrentLimit = FeederConstants.SupplyCurrentLimit;
            rollerMotor.getConfigurator().apply(rollerMotorConfig);
        }
    }

    @Override
    public void periodic() {
        SmartDashboard.putString("STATE/FEEDER WANTED STATE", wantedState.toString());
        SmartDashboard.putString("STATE/FEEDER SYSTEM STATE", systemState.toString());
        SmartDashboard.putNumber("FEEDER/Spindexer Speed", spindexerMotorSpeed);
        SmartDashboard.putNumber("FEEDER/Tower Speed", towerMotorSpeed);
        SmartDashboard.putNumber("FEEDER/Roller Speed", rollerMotorSpeed);
        SmartDashboard.putBoolean("FEEDER/Feeder Shooting", systemState == SystemState.SHOOTING);

        systemState = changeCurrentSystemState();
        applyState();

        if (!Robot.isSimulation()) {
            spindexerMotor.set(spindexerMotorSpeed);
            towerMotor.set(towerMotorSpeed);
            rollerMotor.set(rollerMotorSpeed);
        }
    }
}
