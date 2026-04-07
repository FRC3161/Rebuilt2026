// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.LightsConstants;
import frc.robot.Constants.OperatorConstants;
import frc.robot.Constants.FeederConstants.FeederWantedState;
import frc.robot.Constants.IntakeConstants.IntakeWantedState;
import frc.robot.Constants.IntakeConstants.SystemState;
import frc.robot.Constants.ShooterConstants.ShooterWantedState;
import frc.robot.Constants.TurretConstants.TurretWantedState;
import frc.robot.subsystems.Scoring.Shooter;
import frc.robot.subsystems.Scoring.Turret;
import frc.robot.subsystems.Drive.CommandSwerveDrivetrain;
import frc.robot.subsystems.Drive.TunerConstants;
import frc.robot.subsystems.Intake.Feeder;
import frc.robot.subsystems.Intake.Intake;

import frc.robot.subsystems.Lights.LEDSubsystem_WPIlib;
import frc.robot.subsystems.Lights.LEDSubsystem_WPIlib.LEDTarget;
import frc.robot.commands.Drive.DriveToLocation;
import frc.robot.commands.Intake.HomeIntake;
import frc.robot.commands.Lights.WPIlib.DisableLED;
import frc.robot.commands.Lights.WPIlib.SetTwinklePattern;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.events.EventTrigger;
import com.pathplanner.lib.util.swerve.SwerveSetpointGenerator;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.event.EventLoop;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Robot;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
    public final Timer matchTimer = new Timer();

    // The robot's subsystems and commands are defined here...
    public final LEDSubsystem_WPIlib normalLights = new LEDSubsystem_WPIlib();
    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();
    public final Intake intake = new Intake();
    public final Turret turret = new Turret(drivetrain);
    public final Shooter shooter = new Shooter(drivetrain);
    public final Feeder feeder = new Feeder(turret, shooter, drivetrain);
    // private final Vision vision = new Vision();
    public final MatchInformation matchInformation = new MatchInformation(matchTimer);
    public SendableChooser<Command> sendableChooser = new SendableChooser<>();

    // private double MaxSpeed = 1.0 *
    // TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts
    // desired top speed

    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second
                                                                                      // max angular velocity

    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    // private final SwerveRequest.PointWheelsAt point = new
    // SwerveRequest.PointWheelsAt();

    // Replace with CommandPS4Controller or CommandJoystick if needed
    private final CommandXboxController driver = new CommandXboxController(OperatorConstants.kDriverControllerPort);
    private final CommandXboxController operator = new CommandXboxController(OperatorConstants.kOperatorControllerPort);

    // Simulation only - second driver controller for testing SOTF
    private final CommandXboxController driver2 = Robot.isSimulation() ? new CommandXboxController(2) : null;

    // drive stuff
    private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond);
    private final Telemetry logger = new Telemetry(MaxSpeed);
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1)
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

    private final SwerveRequest.FieldCentricFacingAngle facingAngle = new SwerveRequest.FieldCentricFacingAngle()
            .withDeadband(MaxSpeed * 0.1)
            .withRotationalDeadband(MaxAngularRate * 0.1)
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage)
            .withHeadingPID(5, 0, 0);

    // triggers
    Trigger hubAimLights = new Trigger(() -> turret.getState() == TurretWantedState.AIM_HUB);
    Trigger passsAimLights = new Trigger(() -> turret.getState() == TurretWantedState.AIM_PASS);
    Trigger ecoMode = new Trigger(() -> checkBattery());

    private boolean checkBattery() {
        if (RobotController.getBatteryVoltage() < 10) {
            double check = Timer.getFPGATimestamp() + 4;
            if (Timer.getFPGATimestamp() > check) {
                return true;
            }
            return false;
        }
        return false;
    }

    /**
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */
    public RobotContainer() {
        configureBindings();
        configureNamedCommands();
        configureAutoCommands();
        configureTestCommands();

        if (Robot.isSimulation()) {
            drivetrain.resetPose(new Pose2d(2, 4, Rotation2d.fromDegrees(0)));
        }
    }

    private void configureBindings() {
        drivetrain.registerTelemetry(logger::telemeterize);

        /*********** DRIVER ************/
        drivetrain.setDefaultCommand(
                drivetrain.applyRequest(() -> {
                    double leftY = driver.getLeftY();
                    double leftX = driver.getLeftX();
                    double rightX = driver.getRightX();
                    double slowFactor = operator.rightTrigger().getAsBoolean() ? .3 : 1.0;

                    // Simulation only - driver2 overrides if active
                    if (Robot.isSimulation() && driver2 != null) {
                        if (Math.abs(driver2.getLeftY()) > 0.1)
                            leftY = driver2.getLeftY();
                        if (Math.abs(driver2.getLeftX()) > 0.1)
                            leftX = driver2.getLeftX();
                        if (Math.abs(driver2.getRightX()) > 0.1)
                            rightX = driver2.getRightX();
                        if (driver2.rightTrigger().getAsBoolean())
                            slowFactor = 0.5;
                    }

                    double simTranslationFactor = Robot.isSimulation() ? 0.3 : 1.0;
                    double rawRotation = rightX;

                    return drive
                            .withVelocityX(-leftY * MaxSpeed * slowFactor * simTranslationFactor)
                            .withVelocityY(-leftX * MaxSpeed * slowFactor * simTranslationFactor)
                            .withRotationalRate(-rawRotation * MaxAngularRate * slowFactor);
                }));

        // gyro reset
        driver.start().onTrue(drivetrain.runOnce(drivetrain::seedFieldCentric));

        // Snap Buttons
        driver.y().whileTrue(
                drivetrain.applyRequest(() -> facingAngle
                        .withVelocityX(-driver.getLeftY() * MaxSpeed)
                        .withVelocityY(-driver.getLeftX() * MaxSpeed)
                        .withTargetDirection(Rotation2d.kZero)));

        driver.x().whileTrue(
                drivetrain.applyRequest(() -> facingAngle
                        .withVelocityX(-driver.getLeftY() * MaxSpeed)
                        .withVelocityY(-driver.getLeftX() * MaxSpeed)
                        .withTargetDirection(Rotation2d.kCCW_90deg)));

        driver.b().whileTrue(
                drivetrain.applyRequest(() -> facingAngle
                        .withVelocityX(-driver.getLeftY() * MaxSpeed)
                        .withVelocityY(-driver.getLeftX() * MaxSpeed)
                        .withTargetDirection(Rotation2d.kCW_90deg)));

        driver.a().whileTrue(
                drivetrain.applyRequest(() -> facingAngle
                        .withVelocityX(-driver.getLeftY() * MaxSpeed)
                        .withVelocityY(-driver.getLeftX() * MaxSpeed)
                        .withTargetDirection(Rotation2d.k180deg)));

        // intake
        driver.rightBumper()
                .onTrue(new ConditionalCommand(
                        new InstantCommand(() -> intake.setWantedIntakeState(IntakeWantedState.IDLE)),
                        new InstantCommand(() -> intake.setWantedIntakeState(IntakeWantedState.INTAKE)),
                        () -> intake.getState() == SystemState.INTAKING));
        // .onFalse(new InstantCommand(() ->
        // intake.setWantedIntakeState(IntakeWantedState.IDLE)));

        // retract
        driver.leftBumper()
                .onTrue(new InstantCommand(() -> intake.setWantedIntakeState(IntakeWantedState.RETRACT))) // make this
                                                                                                          // RESET
                .onFalse(new InstantCommand(() -> intake.setWantedIntakeState(IntakeWantedState.IDLE)));

        // outtake
        driver.leftTrigger()
                .onTrue(new InstantCommand(() -> intake.setWantedIntakeState(IntakeWantedState.OUTTAKE)))
                .onFalse(new InstantCommand(() -> intake.setWantedIntakeState(IntakeWantedState.IDLE)));

        // eco mode
        driver.povUp()
                .onTrue(
                        new ParallelCommandGroup(
                                new InstantCommand(() -> intake.enableEcoModeIntake()),
                                new InstantCommand(() -> turret.enableEcoModeTurret()),
                                new InstantCommand(() -> feeder.enableEcoModeFeeder())));
        driver.povDown()
                .onTrue(
                        new ParallelCommandGroup(
                                new InstantCommand(() -> intake.disableEcoModeIntake()),
                                new InstantCommand(() -> turret.disableEcoModeTurret()),
                                new InstantCommand(() -> feeder.disableEcoModeFeeder())));
        // manual intake safety
        // run one way
        driver.povLeft()
                .onTrue(new InstantCommand(() -> intake.setWantedIntakeState(IntakeWantedState.MANUAL_CONTROL_POS)))
                .onFalse(new InstantCommand(() -> intake.setWantedIntakeState(IntakeWantedState.MANUAL_IDLE)));

        // run other way
        driver.povRight()
                .onTrue(new InstantCommand(() -> intake.setWantedIntakeState(IntakeWantedState.MANUAL_CONTROL_NEG)))
                .onFalse(new InstantCommand(() -> intake.setWantedIntakeState(IntakeWantedState.MANUAL_IDLE)));

        // manually set intake to 0
        driver.back().onTrue(new InstantCommand(() -> intake.setZero()));

        // brake
        driver.rightTrigger().whileTrue(drivetrain.applyRequest(() -> brake));
        // driver.b().whileTrue(drivetrain.applyRequest(() ->
        // point.withModuleDirection(new Rotation2d(-driver.getLeftY(),
        // -driver.getLeftX()))
        // ));

        /********* OPERATOR *********/

        // spindexer reverse
        operator.a()
                .onTrue(new InstantCommand(() -> feeder.setWantedFeederState(FeederWantedState.FEEDTEST)))
                .onFalse(new InstantCommand(() -> feeder.setWantedFeederState(FeederWantedState.IDLE)));

        // high pass
        operator.y()
                .onTrue(new SequentialCommandGroup(
                        new InstantCommand(() -> shooter.setWantedShooterState(ShooterWantedState.TEST)),
                        new InstantCommand(() -> turret.setWantedTurretState(TurretWantedState.AIM_PASS)),
                        new InstantCommand(() -> feeder.setWantedFeederState(FeederWantedState.SHOOT))))
                .onFalse(new ParallelCommandGroup(
                        new InstantCommand(() -> shooter.setWantedShooterState(ShooterWantedState.WAIT)),
                        new InstantCommand(() -> turret.setWantedTurretState(TurretWantedState.IDLE)),
                        new InstantCommand(() -> feeder.setWantedFeederState(FeederWantedState.IDLE))));

        // right trench shot
        operator.b()
                .onTrue(new SequentialCommandGroup(
                        new InstantCommand(() -> shooter.setWantedShooterState(ShooterWantedState.TRENCH_SHOOT)),
                        new InstantCommand(() -> turret.setWantedTurretState(TurretWantedState.TRENCH_PRESETR)),
                        new InstantCommand(() -> feeder.setWantedFeederState(FeederWantedState.SHOOT))))
                .onFalse(new ParallelCommandGroup(
                        new InstantCommand(() -> shooter.setWantedShooterState(ShooterWantedState.WAIT)),
                        new InstantCommand(() -> turret.setWantedTurretState(TurretWantedState.IDLE)),
                        new InstantCommand(() -> feeder.setWantedFeederState(FeederWantedState.IDLE))));

        // left trench shot
        operator.x()
                .onTrue(new SequentialCommandGroup(
                        new InstantCommand(() -> shooter.setWantedShooterState(ShooterWantedState.TRENCH_SHOOT)),
                        new InstantCommand(() -> turret.setWantedTurretState(TurretWantedState.TRENCH_PRESETL)),
                        new InstantCommand(() -> feeder.setWantedFeederState(FeederWantedState.SHOOT))))
                .onFalse(new ParallelCommandGroup(
                        new InstantCommand(() -> shooter.setWantedShooterState(ShooterWantedState.WAIT)),
                        new InstantCommand(() -> turret.setWantedTurretState(TurretWantedState.IDLE)),
                        new InstantCommand(() -> feeder.setWantedFeederState(FeederWantedState.IDLE))));

        // agitation manual
        operator.leftTrigger()
                .onTrue((new ParallelCommandGroup(
                        new InstantCommand(() -> intake.setWantedIntakeState(IntakeWantedState.RETRACT)))))
                .onFalse((new ParallelCommandGroup(
                        new InstantCommand(() -> intake.setWantedIntakeState(IntakeWantedState.INTAKE)))));
        // slow squeeze
        operator.leftBumper()
                .onTrue((new ParallelCommandGroup(
                        new InstantCommand(() -> intake.setWantedIntakeState(IntakeWantedState.SCORE)))))
                .onFalse((new ParallelCommandGroup(
                        new InstantCommand(() -> intake.setWantedIntakeState(IntakeWantedState.INTAKE)))));

        // shooting
        operator.rightTrigger()
                .onTrue(new SequentialCommandGroup(
                        new InstantCommand(() -> shooter.setWantedShooterState(ShooterWantedState.HUB_SHOOT)),
                        new InstantCommand(() -> turret.setWantedTurretState(TurretWantedState.AIM_HUB)),
                        new InstantCommand(() -> feeder.setWantedFeederState(FeederWantedState.SHOOT))))
                .onFalse(new ParallelCommandGroup(
                        new InstantCommand(() -> shooter.setWantedShooterState(ShooterWantedState.WAIT)),
                        new InstantCommand(() -> turret.setWantedTurretState(TurretWantedState.IDLE)),
                        new InstantCommand(() -> feeder.setWantedFeederState(FeederWantedState.IDLE))));

        // aiming offset
        operator.povRight()
                .onTrue(new InstantCommand(() -> turret.applyLeftOffset()));
        operator.povLeft()
                .onTrue(new InstantCommand(() -> turret.applyRightOffset()));

        // passing
        operator.rightBumper()
                .onTrue(new SequentialCommandGroup(
                        new InstantCommand(() -> shooter.setWantedShooterState(ShooterWantedState.PASS_SHOOT)),
                        new InstantCommand(() -> turret.setWantedTurretState(TurretWantedState.AIM_PASS)),
                        new InstantCommand(() -> feeder.setWantedFeederState(FeederWantedState.SHOOT))))
                .onFalse(new ParallelCommandGroup(
                        new InstantCommand(() -> shooter.setWantedShooterState(ShooterWantedState.WAIT)),
                        new InstantCommand(() -> turret.setWantedTurretState(TurretWantedState.IDLE)),
                        new InstantCommand(() -> feeder.setWantedFeederState(FeederWantedState.IDLE))));

        /******* CONDITIONAL CONTROLS ***********/

        // Simulation only - driver2 right trigger triggers shooting like operator
        if (Robot.isSimulation() && driver2 != null) {
            driver2.rightTrigger()
                    .onTrue(new SequentialCommandGroup(
                            new InstantCommand(() -> shooter.setWantedShooterState(ShooterWantedState.HUB_SHOOT)),
                            new InstantCommand(() -> turret.setWantedTurretState(TurretWantedState.AIM_HUB)),
                            new InstantCommand(() -> feeder.setWantedFeederState(FeederWantedState.SHOOT))))
                    .onFalse(new ParallelCommandGroup(
                            new InstantCommand(() -> shooter.setWantedShooterState(ShooterWantedState.WAIT)),
                            new InstantCommand(() -> turret.setWantedTurretState(TurretWantedState.IDLE)),
                            new InstantCommand(() -> feeder.setWantedFeederState(FeederWantedState.IDLE))));
        }

        /* TESTING BUTTONS */
    }

    public void disabledActions() {
        feeder.setWantedFeederState(FeederWantedState.IDLE);
        shooter.setWantedShooterState(ShooterWantedState.IDLE);
        turret.setWantedTurretState(TurretWantedState.IDLE);
        intake.setWantedIntakeState(IntakeWantedState.IDLE);

        new SetTwinklePattern(
                normalLights,
                LEDTarget.SIDES,
                LightsConstants.RBGColors.get("black"),
                LightsConstants.RBGColors.get("gold"),
                2.5).schedule();
    }

    public void LEDSHUTOFF() {
        // new DisableLED(normalLights).schedule();
    }

    public void configureTestCommands() {
        SmartDashboard.putData("Intake RESET", new InstantCommand(() -> intake.setZero()));
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        return sendableChooser.getSelected();
    }

    public Command waitToShoot() {
        return Commands.waitUntil(() -> (shooter.shooterIsReady() && turret.turretIsReady()));
    }

    public boolean PLSwaitToShoot() {
        return (shooter.shooterIsReady() && turret.turretIsReady());
    }

    public void configureAutoCommands() {
        sendableChooser = AutoBuilder.buildAutoChooser();
        SmartDashboard.putData("autos", sendableChooser);
    }

    public Command wait(double seconds) {
        return Commands.waitSeconds(seconds);
    }

    public void configureNamedCommands() {
        NamedCommands.registerCommand("Shooter On",
                new InstantCommand(() -> shooter.setWantedShooterState(ShooterWantedState.TURN_ON_AUTO)));

        NamedCommands.registerCommand("Retract Hood",
                new InstantCommand(() -> shooter.setWantedShooterState(ShooterWantedState.RETRACT_AUTO)));

        NamedCommands.registerCommand("FeederIdle",
                new InstantCommand(() -> feeder.setWantedFeederState(FeederWantedState.IDLE)));

        NamedCommands.registerCommand("Auto Trench Shoot Left",
                new SequentialCommandGroup(
                        new InstantCommand(() -> shooter.setWantedShooterState(ShooterWantedState.TRENCH_SHOOT)),
                        new InstantCommand(() -> turret.setWantedTurretState(TurretWantedState.TRENCH_PRESETL)),
                        new InstantCommand(() -> feeder.setWantedFeederState(FeederWantedState.SHOOT)),
                        wait(1.5),
                        new InstantCommand(() -> intake.setWantedIntakeState(IntakeWantedState.RETRACT)))
                        .alongWith(wait(4.5)));

        NamedCommands.registerCommand("Auto Trench Shoot Right",
                new SequentialCommandGroup(
                        new InstantCommand(() -> shooter.setWantedShooterState(ShooterWantedState.TRENCH_SHOOT)),
                        new InstantCommand(() -> turret.setWantedTurretState(TurretWantedState.TRENCH_PRESETR)),
                        new InstantCommand(() -> feeder.setWantedFeederState(FeederWantedState.SHOOT)),
                        wait(1.5),
                        new InstantCommand(() -> intake.setWantedIntakeState(IntakeWantedState.RETRACT)))
                        .alongWith(wait(4.5)));

        NamedCommands.registerCommand("SOTF",
                new SequentialCommandGroup(
                        new InstantCommand(() -> shooter.setWantedShooterState(ShooterWantedState.HUB_SHOOT)),
                        new InstantCommand(() -> turret.setWantedTurretState(TurretWantedState.AIM_HUB)),
                        waitToShoot(),
                        new InstantCommand(() -> feeder.setWantedFeederState(FeederWantedState.SHOOT)),
                        wait(1.5),
                        new InstantCommand(() -> intake.setWantedIntakeState(IntakeWantedState.RETRACT))));

        NamedCommands.registerCommand("Aim Shoot",
                new SequentialCommandGroup(
                        new InstantCommand(() -> shooter.setWantedShooterState(ShooterWantedState.HUB_SHOOT)),
                        new InstantCommand(() -> turret.setWantedTurretState(TurretWantedState.AIM_HUB)),
                        waitToShoot(),
                        new InstantCommand(() -> feeder.setWantedFeederState(FeederWantedState.SHOOT)),
                        wait(1.5),
                        new InstantCommand(() -> intake.setWantedIntakeState(IntakeWantedState.RETRACT)))
                        .alongWith(wait(3.0)));

        NamedCommands.registerCommand("Intake",
                new InstantCommand(() -> intake.setWantedIntakeState(IntakeWantedState.INTAKE)));

        NamedCommands.registerCommand("Rswipe pathFind",
                drivetrain.defer(
                        () -> DriveToLocation.pathFindTo(new Pose2d(8.05, 2.75, Rotation2d.fromDegrees(90)),
                                drivetrain)));

        NamedCommands.registerCommand("Lswipe pathFind",
                drivetrain.defer(
                        () -> DriveToLocation.pathFindTo(new Pose2d(8.05, 5.25, Rotation2d.fromDegrees(90)),
                                drivetrain)));
    }
}