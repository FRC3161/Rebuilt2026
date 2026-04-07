package frc.robot.subsystems.Intake;

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
    private Turret turret;
    private Shooter shooter;
    private CommandSwerveDrivetrain drivetrain;

    /* MOTORS */
    private TalonFX spindexerMotor = new TalonFX(FeederConstants.spindexerMotorID, "rio");
    private TalonFXConfiguration spindexerMotorConfig = new TalonFXConfiguration();
    private TalonFX towerMotor = new TalonFX(FeederConstants.towerMotorID, "rio");
    private TalonFXConfiguration towerMotorConfig = new TalonFXConfiguration();
    private TalonFX rollerMotor = new TalonFX(FeederConstants.rollerMotorID, "rio");
    private TalonFXConfiguration rollerMotorConfig = new TalonFXConfiguration();

    // for velocity control
    private double spindexerMotorSpeed = 0.0;
    private double towerMotorSpeed = 0.0;
    private double rollerMotorSpeed = 0.0;

    /* STATES */
    FeederWantedState wantedState = FeederWantedState.IDLE;
    SystemState systemState = SystemState.IDLING;

    /** Creates a new Feeder */
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
            StatusCode status = StatusCode.StatusCodeNotInitialized;
            for (int i = 0; i < 5; ++i) {
                status = spindexerMotor.getConfigurator().apply(spindexerMotorConfig);
                if (status.isOK())
                    break;
            }
            if (!status.isOK()) {
                System.out.println("Could not apply spindexer configs, error code: " + status.toString());
            }

            for (int i = 0; i < 5; ++i) {
                status = towerMotor.getConfigurator().apply(towerMotorConfig);
                if (status.isOK())
                    break;
            }
            if (!status.isOK()) {
                System.out.println("Could not apply tower configs, error code: " + status.toString());
            }

            for (int i = 0; i < 5; ++i) {
                status = rollerMotor.getConfigurator().apply(rollerMotorConfig);
                if (status.isOK())
                    break;
            }
            if (!status.isOK()) {
                System.out.println("Could not apply roller configs, error code: " + status.toString());
            }
        }
    }

    public void setWantedFeederState(FeederWantedState desiredState) {
        this.wantedState = desiredState;
    }

    private SystemState changeCurrentSystemState() {
        return switch (wantedState) {
            case IDLE:
                yield SystemState.IDLING;
            case INTAKE:
                yield SystemState.INTAKING;
            case SHOOT:
                if (shooter.shooterIsReady() && turret.turretIsReady()) {
                    yield SystemState.SHOOTING;
                } else {
                    yield SystemState.IDLING;
                }
            case PASS:
                yield SystemState.PASSING;
            case FEEDTEST:
                yield SystemState.FEEDTESTING;
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
                if (drivetrain.getPose().getY() > 3.53 && drivetrain.getPose().getY() < 4.53) {
                    spindexerMotorSpeed = 0;
                    towerMotorSpeed = 0;
                    rollerMotorSpeed = 0;
                } else {
                    spindexerMotorSpeed = FeederConstants.feederShootSpeed;
                    towerMotorSpeed = FeederConstants.feederShootSpeed;
                    rollerMotorSpeed = 0.0;
                }
                break; // fix: was missing break, was falling through to FEEDTESTING
            case FEEDTESTING:
                spindexerMotorSpeed = -0.7;
                towerMotorSpeed = -0.7;
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
            spindexerMotor.getConfigurator().apply(spindexerMotorConfig); // fix: was applying towerMotorConfig
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
            spindexerMotor.getConfigurator().apply(spindexerMotorConfig); // fix: was applying towerMotorConfig
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
        SmartDashboard.putBoolean("FEEDER/Feeder Shooting",
                spindexerMotorSpeed == FeederConstants.feederShootSpeed);

        systemState = changeCurrentSystemState();
        applyState();

        if (!Robot.isSimulation()) {
            spindexerMotor.set(spindexerMotorSpeed);
            towerMotor.set(towerMotorSpeed);
            rollerMotor.set(rollerMotorSpeed);
        }
    }
}