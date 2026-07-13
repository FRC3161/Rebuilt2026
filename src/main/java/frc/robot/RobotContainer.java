// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.LightsConstants;
import frc.robot.Constants.OperatorConstants;
import frc.robot.Constants.FeederConstants.FeederWantedState;
import frc.robot.Constants.IntakeConstants.IntakeWantedState;
import frc.robot.Constants.IntakeConstants.SystemState;
import frc.robot.Constants.ShooterConstants.ShooterWantedState;
import frc.robot.Constants.TurretConstants.TurretWantedState;
import frc.robot.commands.Drive.DriveToLocation;
import frc.robot.subsystems.Drive.CommandSwerveDrivetrain;
import frc.robot.subsystems.Drive.TunerConstants;
import frc.robot.subsystems.Intake.Feeder;
import frc.robot.subsystems.Intake.Intake;
import frc.robot.subsystems.Lights.LEDSubsystem_WPIlib;
import frc.robot.subsystems.Lights.LEDSubsystem_WPIlib.LEDTarget;
import frc.robot.subsystems.Scoring.Shooter;
import frc.robot.subsystems.Scoring.Turret;

/**
 * Declares subsystems, button bindings, and autonomous configuration.
 */
public class RobotContainer {
    public final Timer matchTimer = new Timer();

    // Subsystems
    public final LEDSubsystem_WPIlib normalLights = new LEDSubsystem_WPIlib();
    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();
    public final Intake intake = new Intake();
    public final Turret turret = new Turret(drivetrain);
    public final Shooter shooter = new Shooter(drivetrain);
    public final Feeder feeder = new Feeder(turret, shooter, drivetrain);
    public final MatchInformation matchInformation = new MatchInformation(matchTimer);
    private SendableChooser<Command> autoChooser = new SendableChooser<>();

    // Controllers
    private final CommandXboxController driver = new CommandXboxController(OperatorConstants.kDriverControllerPort);
    private final CommandXboxController operator = new CommandXboxController(OperatorConstants.kOperatorControllerPort);
    // Simulation only - second driver controller for testing SOTF
    private final CommandXboxController driver2 = Robot.isSimulation() ? new CommandXboxController(2) : null;

    // Drive requests
    private final double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond);
    private final double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond);
    private final Telemetry logger = new Telemetry(MaxSpeed);

    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1)
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

    private final SwerveRequest.FieldCentricFacingAngle facingAngle = new SwerveRequest.FieldCentricFacingAngle()
            .withDeadband(MaxSpeed * 0.1)
            .withRotationalDeadband(MaxAngularRate * 0.1)
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage)
            .withHeadingPID(5, 0, 0);

    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();

    /* ---------------- Triggers ---------------- */

    // Battery has been below 10V continuously for 4 seconds.
    private static final double kLowBatteryVolts = 10;
    private static final double kLowBatteryDebounceSeconds = 4;
    private double lowBatteryStartTime = -1;

    private boolean batteryIsLow() {
        if (RobotController.getBatteryVoltage() < kLowBatteryVolts) {
            if (lowBatteryStartTime < 0) {
                lowBatteryStartTime = Timer.getFPGATimestamp();
            }
            return Timer.getFPGATimestamp() - lowBatteryStartTime > kLowBatteryDebounceSeconds;
        }
        lowBatteryStartTime = -1;
        return false;
    }

    private final Trigger lowBattery = new Trigger(this::batteryIsLow);
    private final Trigger readyToShoot = new Trigger(
            () -> shooter.shooterIsReady() && turret.turretIsReady());

    public RobotContainer() {
        configureBindings();
        configureNamedCommands();
        configureAutoCommands();
        configureTestCommands();

        if (Robot.isSimulation()) {
            drivetrain.resetPose(new Pose2d(2, 4, Rotation2d.fromDegrees(0)));
        }
    }

    /* ---------------- Command helpers ---------------- */

    /** Sets shooter + turret + feeder states together. */
    private Command scoringState(ShooterWantedState s, TurretWantedState t, FeederWantedState f) {
        return Commands.runOnce(() -> {
            shooter.setWantedShooterState(s);
            turret.setWantedTurretState(t);
            feeder.setWantedFeederState(f);
        });
    }

    /** The standard "release shoot button" state: flywheel stays warm. */
    private Command stopScoring() {
        return scoringState(ShooterWantedState.IDLE, TurretWantedState.IDLE, FeederWantedState.IDLE);
    }

    private Command setIntake(IntakeWantedState state) {
        return Commands.runOnce(() -> intake.setWantedIntakeState(state));
    }

    private Command setEcoMode(boolean enabled) {
        return Commands.runOnce(() -> {
            if (enabled) {
                intake.enableEcoModeIntake();
                turret.enableEcoModeTurret();
                feeder.enableEcoModeFeeder();
            } else {
                intake.disableEcoModeIntake();
                turret.disableEcoModeTurret();
                feeder.disableEcoModeFeeder();
            }
        });
    }

    public Command waitToShoot() {
        return Commands.waitUntil(() -> shooter.shooterIsReady() && turret.turretIsReady());
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

                    return drive
                            .withVelocityX(-leftY * MaxSpeed * slowFactor * simTranslationFactor)
                            .withVelocityY(-leftX * MaxSpeed * slowFactor * simTranslationFactor)
                            .withRotationalRate(-rightX * MaxAngularRate * slowFactor);
                }));

        // gyro reset
        driver.start().onTrue(drivetrain.runOnce(drivetrain::seedFieldCentric));

        // Snap buttons
        driver.y().whileTrue(snapTo(Rotation2d.kZero));
        driver.x().whileTrue(snapTo(Rotation2d.kCCW_90deg));
        driver.b().whileTrue(snapTo(Rotation2d.kCW_90deg));
        driver.a().whileTrue(snapTo(Rotation2d.k180deg));

        // intake toggle
        driver.rightBumper().onTrue(
                Commands.either(
                        setIntake(IntakeWantedState.IDLE),
                        setIntake(IntakeWantedState.INTAKE),
                        () -> intake.getState() == SystemState.INTAKING));

        // retract
        driver.leftBumper()
                .onTrue(setIntake(IntakeWantedState.RETRACT))
                .onFalse(setIntake(IntakeWantedState.IDLE));

        // outtake
        driver.leftTrigger()
                .onTrue(setIntake(IntakeWantedState.OUTTAKE))
                .onFalse(setIntake(IntakeWantedState.IDLE));

        // eco mode (manual)
        driver.povUp().onTrue(setEcoMode(true));
        driver.povDown().onTrue(setEcoMode(false));

        // eco mode (automatic on sustained low battery)
        lowBattery.onTrue(setEcoMode(true));

        // manual intake nudge
        driver.povLeft()
                .onTrue(setIntake(IntakeWantedState.MANUAL_CONTROL_POS))
                .onFalse(setIntake(IntakeWantedState.MANUAL_IDLE));
        driver.povRight()
                .onTrue(setIntake(IntakeWantedState.MANUAL_CONTROL_NEG))
                .onFalse(setIntake(IntakeWantedState.MANUAL_IDLE));

        // manually zero intake extension
        driver.back().onTrue(Commands.runOnce(intake::setZero));

        // brake
        driver.rightTrigger().whileTrue(drivetrain.applyRequest(() -> brake));

        /********* OPERATOR *********/

        // spindexer reverse
        operator.a()
                .onTrue(Commands.runOnce(() -> feeder.setWantedFeederState(FeederWantedState.FEEDTEST)))
                .onFalse(Commands.runOnce(() -> feeder.setWantedFeederState(FeederWantedState.IDLE)));

        // high pass
        operator.y()
                .onTrue(scoringState(ShooterWantedState.TEST, TurretWantedState.AIM_PASS, FeederWantedState.SHOOT))
                .onFalse(stopScoring());

        // right trench shot
        operator.b()
                .onTrue(scoringState(ShooterWantedState.TRENCH_SHOOT, TurretWantedState.TRENCH_PRESETR,
                        FeederWantedState.SHOOT))
                .onFalse(stopScoring());

        // left trench shot
        operator.x()
                .onTrue(scoringState(ShooterWantedState.TRENCH_SHOOT, TurretWantedState.TRENCH_PRESETL,
                        FeederWantedState.SHOOT))
                .onFalse(stopScoring());

        // agitation manual
        operator.leftTrigger()
                .onTrue(setIntake(IntakeWantedState.RETRACT))
                .onFalse(setIntake(IntakeWantedState.INTAKE));

        // slow squeeze
        operator.leftBumper()
                .onTrue(setIntake(IntakeWantedState.SCORE))
                .onFalse(setIntake(IntakeWantedState.INTAKE));

        // hub shot (SOTF)
        operator.rightTrigger()
                .onTrue(scoringState(ShooterWantedState.HUB_SHOOT, TurretWantedState.AIM_HUB,
                        FeederWantedState.SHOOT))
                .onFalse(stopScoring());

        // aiming offset trim
        operator.povRight().onTrue(Commands.runOnce(turret::applyLeftOffset));
        operator.povLeft().onTrue(Commands.runOnce(turret::applyRightOffset));

        // test button
        operator.povDown()
                .onTrue(scoringState(ShooterWantedState.TEST, TurretWantedState.IDLE, FeederWantedState.SHOOT))
                .onFalse(scoringState(ShooterWantedState.IDLE, TurretWantedState.IDLE, FeederWantedState.IDLE));

        // passing
        operator.rightBumper()
                .onTrue(scoringState(ShooterWantedState.PASS_SHOOT, TurretWantedState.AIM_PASS,
                        FeederWantedState.SHOOT))
                .onFalse(stopScoring());

        /******* SIGNAL LIGHTS ***********/

        // Shooter-mounted signal LEDs: green when flywheel + turret are ready
        // to shoot, red otherwise.
        readyToShoot
                .onTrue(Commands.runOnce(
                        () -> normalLights.LED_SolidColor(LEDTarget.SIGNAL,
                                LightsConstants.RBGColors.get("green")),
                        normalLights))
                .onFalse(Commands.runOnce(
                        () -> normalLights.LED_SolidColor(LEDTarget.SIGNAL,
                                LightsConstants.RBGColors.get("red")),
                        normalLights));

        /******* SIMULATION ***********/

        // driver2 right trigger triggers shooting like operator
        if (Robot.isSimulation() && driver2 != null) {
            driver2.rightTrigger()
                    .onTrue(scoringState(ShooterWantedState.HUB_SHOOT, TurretWantedState.AIM_HUB,
                            FeederWantedState.SHOOT))
                    .onFalse(stopScoring());
        }
    }

    private Command snapTo(Rotation2d direction) {
        return drivetrain.applyRequest(() -> facingAngle
                .withVelocityX(-driver.getLeftY() * MaxSpeed)
                .withVelocityY(-driver.getLeftX() * MaxSpeed)
                .withTargetDirection(direction));
    }

    public void disabledActions() {
        feeder.setWantedFeederState(FeederWantedState.IDLE);
        shooter.setWantedShooterState(ShooterWantedState.IDLE);
        turret.setWantedTurretState(TurretWantedState.IDLE);
        intake.setWantedIntakeState(IntakeWantedState.IDLE);

        normalLights.LED_Twinkle(
                LEDTarget.SIDES,
                LightsConstants.RBGColors.get("black"),
                LightsConstants.RBGColors.get("gold"),
                2.5);
    }

    public void teleopActions() {
        // Match timer drives MatchInformation's hub-shift logic; restart (not
        // just start) so re-enables don't inherit stale elapsed time.
        matchTimer.restart();

        feeder.setWantedFeederState(FeederWantedState.IDLE);
        shooter.setWantedShooterState(ShooterWantedState.IDLE);
        turret.setWantedTurretState(TurretWantedState.IDLE);
        intake.setWantedIntakeState(IntakeWantedState.IDLE);

        normalLights.LED_Reset(LEDTarget.SIDES);
    }

    public void configureTestCommands() {
        SmartDashboard.putData("Intake RESET", Commands.runOnce(intake::setZero, intake));
    }

    public Command getAutonomousCommand() {
        return autoChooser.getSelected();
    }

    public void configureAutoCommands() {
        autoChooser = AutoBuilder.buildAutoChooser();
        SmartDashboard.putData("autos", autoChooser);
    }

    public void configureNamedCommands() {
        NamedCommands.registerCommand("Shooter On",
                Commands.runOnce(() -> shooter.setWantedShooterState(ShooterWantedState.TURN_ON_AUTO)));

        NamedCommands.registerCommand("Retract Hood",
                Commands.runOnce(() -> shooter.setWantedShooterState(ShooterWantedState.RETRACT_AUTO)));

        NamedCommands.registerCommand("FeederIdle",
                Commands.runOnce(() -> feeder.setWantedFeederState(FeederWantedState.IDLE)));

        NamedCommands.registerCommand("Auto Trench Shoot Left",
                Commands.sequence(
                        scoringState(ShooterWantedState.TRENCH_SHOOT, TurretWantedState.TRENCH_PRESETL,
                                FeederWantedState.SHOOT),
                        Commands.waitSeconds(1.5),
                        setIntake(IntakeWantedState.RETRACT))
                        .alongWith(Commands.waitSeconds(4.5)));

        NamedCommands.registerCommand("Auto Trench Shoot Right",
                Commands.sequence(
                        scoringState(ShooterWantedState.TRENCH_SHOOT, TurretWantedState.TRENCH_PRESETR,
                                FeederWantedState.SHOOT),
                        Commands.waitSeconds(1.5),
                        setIntake(IntakeWantedState.RETRACT))
                        .alongWith(Commands.waitSeconds(4.5)));

        NamedCommands.registerCommand("SOTF",
                Commands.sequence(
                        scoringState(ShooterWantedState.HUB_SHOOT, TurretWantedState.AIM_HUB,
                                FeederWantedState.IDLE),
                        waitToShoot(),
                        Commands.runOnce(() -> feeder.setWantedFeederState(FeederWantedState.SHOOT)),
                        Commands.waitSeconds(1.5),
                        setIntake(IntakeWantedState.RETRACT)));

        NamedCommands.registerCommand("Aim Shoot",
                Commands.sequence(
                        scoringState(ShooterWantedState.HUB_SHOOT, TurretWantedState.AIM_HUB,
                                FeederWantedState.IDLE),
                        waitToShoot(),
                        Commands.runOnce(() -> feeder.setWantedFeederState(FeederWantedState.SHOOT)),
                        Commands.waitSeconds(1.5),
                        setIntake(IntakeWantedState.RETRACT))
                        .alongWith(Commands.waitSeconds(3.0)));

        NamedCommands.registerCommand("Intake",
                setIntake(IntakeWantedState.INTAKE));

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
