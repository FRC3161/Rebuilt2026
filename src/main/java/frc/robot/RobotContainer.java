// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.LightsConstants;
import frc.robot.Constants.OperatorConstants;
import frc.robot.Constants.FeederConstants.FeederWantedState;
import frc.robot.Constants.IntakeConstants.IntakeWantedState;
import frc.robot.Constants.ShooterConstants.ShooterWantedState;
import frc.robot.Constants.TurretConstants.TurretWantedState;
import frc.robot.subsystems.Scoring.Shooter;
import frc.robot.subsystems.Scoring.Turret;
import frc.robot.subsystems.Drive.CommandSwerveDrivetrain;
import frc.robot.subsystems.Drive.TunerConstants;
import frc.robot.subsystems.Intake.Feeder;
import frc.robot.subsystems.Intake.Intake;

import frc.robot.subsystems.Lights.LEDSubsystem_WPIlib;
import frc.robot.commands.Lights.WPIlib.RunPattern;
import frc.robot.commands.Lights.WPIlib.ScrollPattern;
import frc.robot.commands.Lights.WPIlib.SetBlinkingPattern;
import frc.robot.commands.Lights.WPIlib.SetBreathingPattern;
import frc.robot.commands.Lights.WPIlib.SetSolidColor;
import frc.robot.commands.Lights.WPIlib.SetTwinklePattern;
import frc.robot.commands.Lights.WPIlib.DisableLED;
import frc.robot.commands.Lights.WPIlib.ResetLED;

import static edu.wpi.first.units.Units.Meter;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import java.util.jar.Attributes.Name;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.LEDPattern.GradientType;
import edu.wpi.first.wpilibj.event.EventLoop;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final LEDSubsystem_WPIlib normalLights = new LEDSubsystem_WPIlib();
  public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();
  private final Intake intake = new Intake();
  private final Turret turret = new Turret(drivetrain, normalLights);
  private final Shooter shooter = new Shooter(drivetrain);
    private final Feeder feeder = new Feeder(turret, shooter);
  // private final Vision vision = new Vision();
  private final MatchInformation matchInformation = new MatchInformation(normalLights);
  public SendableChooser<Command> sendableChooser = new SendableChooser<>();
  


  // private double MaxSpeed = 1.0 * TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed

  private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity
  
  private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
  private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final CommandXboxController driver = new CommandXboxController(OperatorConstants.kDriverControllerPort);
  private final CommandXboxController operator = new CommandXboxController(OperatorConstants.kOperatorControllerPort);

  // drive stuff
  private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
  private final Telemetry logger = new Telemetry(MaxSpeed);
  private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
                        .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1)
                        .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive

  private final SwerveRequest.FieldCentricFacingAngle facingAngle = new SwerveRequest.FieldCentricFacingAngle()
                        .withDeadband(MaxSpeed * 0.1)
                        .withRotationalDeadband(MaxAngularRate * 0.1)
                        .withDriveRequestType(DriveRequestType.OpenLoopVoltage)
                        .withHeadingPID(6, 0, 0);

  // triggers
  Trigger activeHubWarning = new Trigger(() -> matchInformation.warningLight() == "G");
  Trigger inactiveHubWarning = new Trigger(() -> matchInformation.warningLight() == "R");
  Trigger hubAimLights = new Trigger(() -> turret.getState() == TurretWantedState.AIM_HUB);
  Trigger passsAimLights = new Trigger(() -> turret.getState() == TurretWantedState.AIM_PASS);


  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the trigger bindings
    configureBindings();
    configureNamedCommands();
    configureAutoCommands();
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {
    drivetrain.registerTelemetry(logger::telemeterize);
    
    drivetrain.setDefaultCommand(
        // Drivetrain will execute this command periodically
        drivetrain.applyRequest(() -> {

            return drive.withVelocityX(-driver.getLeftY() * MaxSpeed) // Drive forward with negative Y (forward)
                .withVelocityY(-driver.getLeftX() * MaxSpeed ) // Drive left with negative X (left)
                .withRotationalRate(-driver.getRightX() * MaxAngularRate); // Drive counterclockwise with negative X (left)
  })
    );
    //gyro reset
    driver.start().onTrue(drivetrain.runOnce(drivetrain::seedFieldCentric));
    
    //Snap Buttons
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
    
    //intake
    driver.rightBumper()
      .onTrue(new InstantCommand(() -> intake.setWantedIntakeState(IntakeWantedState.INTAKE)))
      .onFalse(new InstantCommand(() -> intake.setWantedIntakeState(IntakeWantedState.IDLE)));
    //retract
    driver.leftBumper()
      .onTrue(new InstantCommand(() -> intake.setWantedIntakeState(IntakeWantedState.RETRACT)))
      .onFalse(new InstantCommand(() -> intake.setWantedIntakeState(IntakeWantedState.IDLE)));
    //outtake
    driver.leftTrigger()
      .onTrue(new InstantCommand(() -> intake.setWantedIntakeState(IntakeWantedState.OUTTAKE)))
      .onFalse(new InstantCommand(() -> intake.setWantedIntakeState(IntakeWantedState.IDLE)));

    // Brake
    // driver.a().whileTrue(drivetrain.applyRequest(() -> brake));
    // driver.b().whileTrue(drivetrain.applyRequest(() ->
    //     point.withModuleDirection(new Rotation2d(-driver.getLeftY(), -driver.getLeftX()))
    // ));  

    /* OPERATOR */
    //trench shot
    operator.b()
      .onTrue(new SequentialCommandGroup(
        new InstantCommand(() -> shooter.setWantedShooterState(ShooterWantedState.TRENCH_SHOOT)),
        new InstantCommand(() -> turret.setWantedTurretState(TurretWantedState.TRENCH_PRESETL)),
        // waitToShoot(),
        new InstantCommand(() -> feeder.setWantedFeederState(FeederWantedState.SHOOT))))
      .onFalse(new ParallelCommandGroup(
        new InstantCommand(() -> shooter.setWantedShooterState(ShooterWantedState.IDLE)),
        new InstantCommand(() -> turret.setWantedTurretState(TurretWantedState.IDLE)),
        new InstantCommand(() -> feeder.setWantedFeederState(FeederWantedState.IDLE))));
    //agitation manual
    operator.leftTrigger()
      .onTrue((new ParallelCommandGroup(
        new InstantCommand(() -> intake.setWantedIntakeState(IntakeWantedState.RETRACT)))))
      .onFalse((new ParallelCommandGroup(
        new InstantCommand(() -> intake.setWantedIntakeState(IntakeWantedState.INTAKE)))));
    // shooting
    operator.rightTrigger()
      .onTrue(new SequentialCommandGroup(
          new InstantCommand(() -> shooter.setWantedShooterState(ShooterWantedState.HUB_SHOOT)),
          new InstantCommand(() -> turret.setWantedTurretState(TurretWantedState.AIM_HUB)),
          // waitToShoot(),
          new InstantCommand(() -> feeder.setWantedFeederState(FeederWantedState.SHOOT)))
      )
      .onFalse(new ParallelCommandGroup(
        new InstantCommand(() -> shooter.setWantedShooterState(ShooterWantedState.IDLE)),
        new InstantCommand(() -> turret.setWantedTurretState(TurretWantedState.IDLE)),
        new InstantCommand(() -> feeder.setWantedFeederState(FeederWantedState.IDLE))));
    // passing
    operator.rightBumper()
      .onTrue(new SequentialCommandGroup(
          new InstantCommand(() -> shooter.setWantedShooterState(ShooterWantedState.PASS_SHOOT)),
          new InstantCommand(() -> turret.setWantedTurretState(TurretWantedState.AIM_PASS)),
          // waitToShoot(),
          new InstantCommand(() -> feeder.setWantedFeederState(FeederWantedState.SHOOT))
      ))
      .onFalse(new ParallelCommandGroup(
        new InstantCommand(() -> shooter.setWantedShooterState(ShooterWantedState.IDLE)),
        new InstantCommand(() -> turret.setWantedTurretState(TurretWantedState.IDLE)),
        new InstantCommand(() -> feeder.setWantedFeederState(FeederWantedState.IDLE))));

    /* CONDITIONAL CONTROLS */
    activeHubWarning
      .onTrue(new SetBlinkingPattern(normalLights, LEDPattern.solid(LightsConstants.RBGColors.get("green")), 0.5, 0.5))
      .onFalse(new DisableLED(normalLights));
    
    inactiveHubWarning
      .onTrue(new SetBlinkingPattern(normalLights, LEDPattern.solid(LightsConstants.RBGColors.get("red")), 0.5, 0.5))
      .onFalse(new DisableLED(normalLights));

    hubAimLights
      .onTrue(new ScrollPattern(normalLights, LEDPattern.gradient(GradientType.kContinuous, Color.kCadetBlue, Color.kLightGreen), 2.5))
      .onFalse(new DisableLED(normalLights));

    passsAimLights
      .onTrue(new ScrollPattern(normalLights, LEDPattern.gradient(GradientType.kContinuous, Color.kOrange, Color.kYellow), 2.5))
      .onFalse(new DisableLED(normalLights));

      /* TESTING BUTTONS */
    //turret
    // operator.x()
    //   .onTrue(new InstantCommand(() -> turret.setWantedTurretState(TurretWantedState.AIM_HUB)))
    //   .onFalse(new InstantCommand(() -> turret.setWantedTurretState(TurretWantedState.IDLE)));
    // operator.b()
    //   .onTrue(new SequentialCommandGroup(
    //     new InstantCommand(() -> shooter.setWantedShooterState(ShooterWantedState.TRENCH_SHOOT)),
    //     new InstantCommand(() -> turret.setWantedTurretState(TurretWantedState.TRENCH_PRESET)),
    //     waitToShoot(),
    //     new InstantCommand(() -> feeder.setWantedFeederState(FeederWantedState.SHOOT))))
    //   .onFalse(new ParallelCommandGroup(
    //     new InstantCommand(() -> shooter.setWantedShooterState(ShooterWantedState.IDLE)),
    //     new InstantCommand(() -> turret.setWantedTurretState(TurretWantedState.IDLE)),
    //     new InstantCommand(() -> feeder.setWantedFeederState(FeederWantedState.IDLE))));
    
    // 
    // operator.a()
    //   .onTrue(new SequentialCommandGroup(
    //       new InstantCommand(() -> shooter.setWantedShooterState(ShooterWantedState.HUB_SHOOT)),
    //       waitToShoot(),
    //       new InstantCommand(() -> feeder.setWantedFeederState(FeederWantedState.SHOOT))
    //   ))
    //   .onFalse(new ParallelCommandGroup(
    //       new InstantCommand(() -> shooter.setWantedShooterState(ShooterWantedState.IDLE)),
    //       new InstantCommand(() -> feeder.setWantedFeederState(FeederWantedState.IDLE))
    //   ));

    // operator.rightTrigger()
    //   .onTrue(new SequentialCommandGroup(
    //     new InstantCommand(() -> shooter.setWantedShooterState(ShooterWantedState.TEST)),
    //       waitToShoot(),
    //       new InstantCommand(() -> feeder.setWantedFeederState(FeederWantedState.SHOOT))))
    //   .onFalse(new SequentialCommandGroup(
    //     new InstantCommand(() -> shooter.setWantedShooterState(ShooterWantedState.IDLE)),
    //       new InstantCommand(() -> feeder.setWantedFeederState(FeederWantedState.IDLE))));

    // //feeder
    
    // //lights
    // operator.povUp()
    //   .onTrue(new SetBreathingPattern(normalLights, LEDPattern.rainbow(255, 150), 3));
    // operator.povDown()
    //   .onTrue(new ResetLED(normalLights));

    /* UNNEEDED, DELETE */
    // Idle while the robot is disabled. This ensures the configured
    // neutral mode is applied to the drive motors while disabled.
    // final var idle = new SwerveRequest.Idle();
    // RobotModeTriggers.disabled().whileTrue(
    //     drivetrain.applyRequest(() -> idle).ignoringDisable(true)
    // );
    // Run SysId routines when holding back/start and X/Y.
    // Note that each routine should be run exactly once in a single log.
    // joystick.back().and(joystick.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
    // joystick.back().and(joystick.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
    // joystick.start().and(joystick.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
    // joystick.start().and(joystick.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

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
    return  (shooter.shooterIsReady() && turret.turretIsReady());
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
        waitToShoot(),
        new InstantCommand(() -> feeder.setWantedFeederState(FeederWantedState.SHOOT)),
        wait(1.0),
        new InstantCommand(() -> intake.setWantedIntakeState(IntakeWantedState.RETRACT))).withTimeout(3.5));

         NamedCommands.registerCommand("Auto Trench Shoot Right", 
      new SequentialCommandGroup(
        new InstantCommand(() -> shooter.setWantedShooterState(ShooterWantedState.TRENCH_SHOOT)),
        new InstantCommand(() -> turret.setWantedTurretState(TurretWantedState.TRENCH_PRESETR)),
        waitToShoot(),
        new InstantCommand(() -> feeder.setWantedFeederState(FeederWantedState.SHOOT)),
        wait(1.0),
        new InstantCommand(() -> intake.setWantedIntakeState(IntakeWantedState.RETRACT))).withTimeout(3.5));

    NamedCommands.registerCommand("Aim Shoot", 
      new SequentialCommandGroup(
        new InstantCommand(() -> shooter.setWantedShooterState(ShooterWantedState.HUB_SHOOT)),
        new InstantCommand(() -> turret.setWantedTurretState(TurretWantedState.AIM_HUB)),
        waitToShoot(),
        new InstantCommand(() -> feeder.setWantedFeederState(FeederWantedState.SHOOT)),
        wait(1.5),
        new InstantCommand(() -> intake.setWantedIntakeState(IntakeWantedState.RETRACT))));

    NamedCommands.registerCommand("Intake", 
      new InstantCommand(() -> intake.setWantedIntakeState(IntakeWantedState.INTAKE)));
  }
    

}
