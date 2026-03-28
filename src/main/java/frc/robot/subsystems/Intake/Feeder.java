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

    // for velocity control
    private double spindexerMotorSpeed = 0.0;
    private double towerMotorSpeed = 0.0;

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
                break;
            case INTAKING:
                spindexerMotorSpeed = FeederConstants.feederIntakeSpeed;
                towerMotorSpeed = 0.0;
                break;
            case SHOOTING:
                spindexerMotorSpeed = FeederConstants.feederShootSpeed;
                towerMotorSpeed = FeederConstants.feederShootSpeed;
                break;
        case PASSING:
    if (drivetrain.getPose().getY() > 3.53 && drivetrain.getPose().getY() < 4.53) {
        spindexerMotorSpeed = 0;
        towerMotorSpeed = 0;
    } else if ((drivetrain.getPose().getX() > 3.75 && drivetrain.getPose().getX() < 5.0)
            || (drivetrain.getPose().getX() > 11.2 && drivetrain.getPose().getX() < 12.55)) {
        spindexerMotorSpeed = 0;
        towerMotorSpeed = 0;
    } else {
        spindexerMotorSpeed = FeederConstants.feederShootSpeed;
        towerMotorSpeed = FeederConstants.feederShootSpeed;
    }
    break;
            case FEEDTESTING:
                spindexerMotorSpeed = -0.7;
                towerMotorSpeed = -0.7;
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
        }
    }

    @Override
    public void periodic() {
        SmartDashboard.putString("FEEDER WANTED STATE", wantedState.toString());
        SmartDashboard.putString("FEEDER SYSTEM STATE", systemState.toString());
        SmartDashboard.putNumber("Spindexer Speed", spindexerMotorSpeed);
        SmartDashboard.putNumber("Tower Speed", towerMotorSpeed);
        SmartDashboard.putBoolean("Feeder Shooting", 
            spindexerMotorSpeed == FeederConstants.feederShootSpeed);

        systemState = changeCurrentSystemState();
        applyState();

        if (!Robot.isSimulation()) {
            spindexerMotor.set(spindexerMotorSpeed);
            towerMotor.set(towerMotorSpeed);
        }
    }
}