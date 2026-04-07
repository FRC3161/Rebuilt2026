// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.Optional;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.util.Point;
import frc.util.PolynomialRegression;

import java.util.Arrays;
// Lights
import java.util.Map;
import static edu.wpi.first.units.Units.Meters;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.util.Color;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static Optional<Alliance> alliance = DriverStation.getAlliance();

  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
    public static final int kOperatorControllerPort = 1;
  }

  public static class ShooterConstants {
    public static double activeWaitingSpeed = 30;
    public static double inactiveWaitingSpeed;
    public static PolynomialRegression hoodAngleInterpolation = 
    new PolynomialRegression(
      Arrays.asList(
        new Point(2, 0),
        new Point(3, 3),
        new Point(4, 5.5),
        new Point(5, 6), // 6.3
        new Point(6, 7) //7.5
      ),
       2);
    
    public static PolynomialRegression shooterSpeedInterpolation = 
    new PolynomialRegression(
      Arrays.asList(
        new Point(2, 47),
        new Point(3, 50),
        new Point(4, 50),
        new Point(5, 55),
        new Point(6, 62)
      ),
       2);
    public static PolynomialRegression timeOfFlightInterpolation = 
    new PolynomialRegression(
      Arrays.asList(
        new Point(2, .975),
        new Point(3, 1.125),
        new Point(4, 1.175),
        new Point(5, 1.18),
        new Point(6, 1.28)
      ),
       2);
    public static double distanceToHub;
    public static double passDistance;

    public static double shooterMotionMagicExpoK_V = 0.1;
    public static double shooterMotionMagicExpoK_A = 0.1;
    public static double shooterMotionMagicAccel = 400; //rps^2
    public static double shooterMotionMagicJerk = 4000; //rps^2/s

    public static int SupplyCurrentLimit = 80;
    public static int StatorCurrentLimit = 80;
    public static double homingThreshold;
    public static double tolerance = 5;

    public static int hoodMotorID = 53;
    public static int shooterMotor1ID = 61;
    public static int shooterMotor2ID = 60;
    
    public static double[] hoodPID = {2.0, 0, 0};
    public static double[] hoodSVA = {0.0, 0, 0};

    public static double[] shooterPID = {0.6, 0, 0};
    public static double[] shooterSVA = {0.0, 0.117, 0};
    public enum ShooterWantedState {
      IDLE,
      WAIT,
      TRENCH_SHOOT,
      PASS_SHOOT,
      HUB_SHOOT,
      HOME,
      TEST,
      RETRACT_AUTO,
      TURN_ON_AUTO
    }
    public enum SystemState {
      IDLING,
      ACTIVE_WAITING,
      INACTIVE_WAITING,
      TRENCH_SHOOTING,
      PASS_SHOOTING,
      HUB_SHOOTING,
      HOMING,
      TESTING,
      RETRACTING_AUTO,
      TURNING_ON_AUTO
    }
  }

  public static class IntakeConstants {
    public static double intakeMotionMagicExpoK_V = 0.13;
    public static double intakeMotionMagicExpoK_A = 0.1;
    public static double intakeMotionMagicAccel;
    public static double intakeMotionMagicCruiseVel;

    public static int SupplyCurrentLimit = 60;
    public static int StatorCurrentLimit = 60;

    public static int intakeMotorID = 32;
    public static int intakeExtensionMotorID = 31;

    public static double intakingMAXPosition = 10.022461;
    public static double intakingPosition = 10;
    public static double intakingSpeed = -0.8;
    public static double shootingPosition;
    public static double retractingPos = 0;
    public static double[] intakePID = {0.3, 0, 0};
    public static double[] intakeSVA = {0, 0.13, 0.01};
    public enum IntakeWantedState {
      IDLE,
      INTAKE, 
      RETRACT,
      RESET,
      SCORE,
      OUTTAKE
    }
    public enum SystemState {
      IDLING,
      INTAKING,
      RETRACTING,
      RESETING,
      SCORING,
      OUTTAKING
    }
  }  
  
  
  public static class TurretConstants {
    public static int turretMotionMagicExpoK_V;
    public static int turretMotionMagicExpoK_A;
    public static int turretMotionMagicAccel;
    public static int turretMotionMagicJerk;

    public static int SupplyCurrentLimit = 60;
    public static int StatorCurrentLimit = 60;

    public static int turretMotorID = 50;
    public static int encoderID = 54;

    public static double passAimPosition = 0;
    public static double hubPresetPosition;
    public static double trenchPresetPositionL = .52;
    public static double trenchPresetPositionR = .50;
    public static double tolerance = 0.01;  //0.007

    public static double[] turretPID = {51, 0, 0};
    public static double[] turretSVA = {0, 0, 0};

    public enum TurretWantedState {
      IDLE,
      IDLE_AIM,
      AIM_PASS,
      AIM_HUB,
      TRENCH_PRESETL,
      TRENCH_PRESETR,
      HUB_PRESET,
      TEST
      
    }
    public enum SystemState {
      IDLING,
      IDLE_AIMING,
      PASS_AIMING,
      HUB_AIMING,
      TRENCH_PRESETTINGL,
      TRENCH_PRESETTINGR,
      HUB_PRESETTING,
      TESTING
    }
  }

  public static class FeederConstants {
    public static int feederMotionMagicExpoK_V;
    public static int feederMotionMagicExpoK_A;
    public static int feederMotionMagicAccel;
    public static int feederMotionMagicJerk;
    
    public static int SupplyCurrentLimit = 60;
    public static int StatorCurrentLimit = 60;
    
    public static int towerMotorID = 41;
    public static int spindexerMotorID = 40;

    public static double feederIntakeSpeed = 0;
    public static double feederShootSpeed = 0.8;
    
    public static double[] feederPID = {0, 0, 0};
    public static double[] feederSVA = {0, 0, 0};
    
    public enum FeederWantedState {
      IDLE,
      INTAKE,
      SHOOT,
      PASS,
      FEEDTEST
    }
    public enum SystemState {
      IDLING,
      INTAKING,
      SHOOTING,
      PASSING,
      FEEDTESTING
    }
  }

  public static class LightsConstants {
    public static Distance spacing = Meters.of(1 / 60);   // (1 / 60) - 60 leds per 1m strip [Spacing: 1m/#ofLEDs]
    // LED Strip
    public static int led_port = 0;
    public static int led_length = 19;   // 48 LEDs [controllers], 24 a side
    public static int led_brightness = 10;
    // Signal LED Sector (on shooter)
    public static int signal_length = 4;   // 10

    public static enum LightsType {
        ENDGAME,
        CLIMB,
        SHOOTING,
        INTAKE,
        IDLE,
        DISABLED
      }

    public static class Colors {
        public static int[] RED = new int[] { 255, 0, 0 };
        public static int[] GREEN = new int[] { 0, 255, 0 };
        public static int[] BLUE = new int[] { 0, 0, 255 };
        public static int[] GOLD = new int[] { 175, 184, 6 };
        public static int[] MAGENTA = new int[] { 255, 0, 255 };
        public static int[] BRIGHT = new int[] { 234, 255, 48 };
      }

    // RGB Color Map
    // Not needed: use Color.k[colorname]
    public static Map<String, Color> RGBColors = Map.ofEntries(
        Map.entry("black", new Color(0, 0, 0)),
        Map.entry("white", new Color(255, 255, 255)),
        Map.entry("red", new Color(255, 0, 0)),
        Map.entry("green", new Color(0, 255, 0)),
        Map.entry("blue", new Color(0, 0, 255)),
        Map.entry("gold", new Color(175, 184, 6)),
        Map.entry("team_Gold", new Color(179, 134, 27)),
        Map.entry("yellow", new Color(255, 255, 0)),
        Map.entry("orange", new Color(255, 165, 0)),
        Map.entry("pink", new Color(255, 20, 147)),
        Map.entry("magenta", new Color(255, 0, 255)),
        Map.entry("bright", new Color(234, 255, 48)));

    // RBG Color Map (New LEDs)
    public static Map<String, Color> RBGColors = Map.ofEntries(
        Map.entry("black", new Color(0, 0, 0)),
        Map.entry("white", new Color(255, 255, 255)),
        Map.entry("red", new Color(255, 0, 0)),
        Map.entry("green", new Color(0, 0, 255)),
        Map.entry("blue", new Color(0, 255, 0)),
        Map.entry("gold", new Color(175, 6, 184)),
        Map.entry("team_Gold", new Color(179, 27, 134)),
        Map.entry("yellow", new Color(255, 0, 255)),
        Map.entry("orange", new Color(255, 0, 165)),
        Map.entry("pink", new Color(255, 147, 20)),
        Map.entry("magenta", new Color(255, 255, 0)),
        Map.entry("bright", new Color(234, 48, 255)));

    // GRB Color Map (Old LEDs)
    public static Map<String, Color> GRBColors = Map.ofEntries(
        Map.entry("black", new Color(0, 0, 0)),
        Map.entry("white", new Color(255, 255, 255)),
        Map.entry("red", new Color(0, 255, 0)),
        Map.entry("green", new Color(255, 0, 0)),
        Map.entry("blue", new Color(0, 0, 255)),
        Map.entry("gold", new Color(184, 175, 6)),
        Map.entry("team_Gold", new Color(134, 179, 27)),
        Map.entry("yellow", new Color(255, 255, 0)),
        Map.entry("orange", new Color(165, 255, 0)),
        Map.entry("pink", new Color(20, 255, 147)),
        Map.entry("magenta", new Color(0, 255, 255)),
        Map.entry("bright", new Color(255, 234, 48)));

    // GBR Color Map
    public static Map<String, Color> GBRColors = Map.ofEntries(
        Map.entry("black", new Color(0, 0, 0)),
        Map.entry("white", new Color(255, 255, 255)),
        Map.entry("red", new Color(0, 0, 255)),
        Map.entry("green", new Color(255, 0, 0)),
        Map.entry("blue", new Color(0, 255, 0)),
        Map.entry("gold", new Color(184, 6, 175)),
        Map.entry("team_Gold", new Color(134, 27, 179)),
        Map.entry("yellow", new Color(255, 0, 255)),
        Map.entry("orange", new Color(165, 0, 255)),
        Map.entry("pink", new Color(20, 147, 255)),
        Map.entry("magenta", new Color(0, 255, 255)),
        Map.entry("bright", new Color(255, 48, 234)));

    // BRG Color Map
    public static Map<String, Color> BRGColors = Map.ofEntries(
        Map.entry("black", new Color(0, 0, 0)),                 // (0,0,0) → (0,0,0)
        Map.entry("white", new Color(255, 255, 255)),           // (255,255,255) → (255,255,255)
        Map.entry("red", new Color(0, 255, 0)),                 // RGB(255,0,0) → BRG(0,255,0)
        Map.entry("green", new Color(0, 0, 255)),               // RGB(0,255,0) → BRG(0,0,255)
        Map.entry("blue", new Color(255, 0, 0)),                // RGB(0,0,255) → BRG(255,0,0)
        Map.entry("gold", new Color(6, 175, 184)),              // RGB(175,184,6) → BRG(6,175,184)
        Map.entry("team_Gold", new Color(27, 179, 134)),        // RGB(179,134,27) → BRG(27,179,134)
        Map.entry("yellow", new Color(0, 255, 255)),            // RGB(255,255,0) → BRG(0,255,255)
        Map.entry("orange", new Color(0, 255, 165)),            // RGB(255,165,0) → BRG(0,255,165)
        Map.entry("pink", new Color(147, 255, 20)),             // RGB(255,20,147) → BRG(147,255,20)
        Map.entry("magenta", new Color(255, 255, 0)),           // RGB(255,0,255) → BRG(255,255,0)
        Map.entry("bright", new Color(48, 234, 255)));          // RGB(234,255,48) → BRG(48,234,255)
  }

  public static class VisionConstants {
    public static Transform2d turretToCenter = 
      new Transform2d(
        Units.inchesToMeters(-6.5), 
        Units.inchesToMeters(-6), 
        new Rotation2d());

    public static Translation2d BLUE_HUB_POSE =
      new Translation2d(4.62, 4.03); //was 4.03
    public static Translation2d RED_HUB_POSE =
      new Translation2d(12, 3.85); //was 11.92, 4.03

    public static Translation2d BLUE_PASS_SPOT_1 = 
      new Translation2d(1, 1);
    public static Translation2d BLUE_PASS_SPOT_2 = 
      new Translation2d(1, 7);
    public static Translation2d RED_PASS_SPOT_1 = 
      new Translation2d(14.5, 7); //15.5
    public static Translation2d RED_PASS_SPOT_2 = 
      new Translation2d(14.5, 1); //15.5

    public static double bumperToBumper; // inches

    public static Transform3d kRobotToCam = new Transform3d(
      new Translation3d(
        (Units.inchesToMeters(6.643)), 
        -Units.inchesToMeters(0.616), 
        Units.inchesToMeters(17.467+2.7525)),
      new Rotation3d(
        0, 
        Units.degreesToRadians(20), 
        0)
      );
    // center back cam
    public static Transform3d kRobotToCam2 = new Transform3d(
      new Translation3d(
        -(Units.inchesToMeters(12.844)), 
        Units.inchesToMeters(0.848),
        Units.inchesToMeters(12.195)),
      new Rotation3d(
        0, 
        Units.degreesToRadians(20), 
        Units.degreesToRadians(135))
        );
    // corner camera
    public static Transform3d kRobotToCam3 = new Transform3d(
      new Translation3d(
        -(Units.inchesToMeters(12.843)), 
        -(Units.inchesToMeters(12.851)),
        Units.inchesToMeters(12.195)),
      new Rotation3d(
        0, 
        Units.degreesToRadians(20), 
        Units.degreesToRadians(225))
        );

    public static String cameraName = "camera1";
    public static String camera2Name = "camera2";
    public static String camera3Name = "camera3";

    /* standard deviations for vision calculations */
    public static edu.wpi.first.math.Vector<N3> kSingleTagStdDevs = VecBuilder.fill(4, 4, 4);
    public static edu.wpi.first.math.Vector<N3> kMultiTagStdDevs = VecBuilder.fill(2.5 , 2.5, 2.5);
    public static edu.wpi.first.math.Vector<N3> odoStdDEvs = VecBuilder.fill(.2, .2, .05);
    public static double odometryUpdateFrequency = 250;
  }

  public static class FieldConstants {
    public enum ScoringZone {
      RED_PASSING_1,
      RED_PASSING_2,
      BLUE_PASSING_1,
      BLUE_PASSING_2,
      RED_HUB,
      BLUE_HUB,
      NO_TRACK
    }
  }
}
