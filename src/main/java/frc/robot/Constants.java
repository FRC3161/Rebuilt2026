// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.Meters;

import java.util.Map;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.util.Color;

/**
 * Robot-wide constants. All values are public static; nothing in this class
 * should be functional.
 *
 * NOTE: alliance color must always be read live via
 * DriverStation.getAlliance() — never cache it in a static field, since it is
 * empty until the Driver Station connects.
 */
public final class Constants {

    public static class OperatorConstants {
        public static final int kDriverControllerPort = 0;
        public static final int kOperatorControllerPort = 1;
    }

    public static class AutoConstants {
        // Pathfinding constraints. Drivetrain free speed is ~4.58 m/s
        // (TunerConstants.kSpeedAt12Volts) — keep max speed below that.
        public static final double kMaxSpeedMetersPerSecond = 4.0; // TODO: tune
        public static final double kMaxAccelerationMetersPerSecondSquared = 3.0; // TODO: tune
        public static final double kMaxAngularSpeedRadiansPerSecond = 2 * Math.PI; // TODO: tune
        public static final double kMaxAngularSpeedRadiansPerSecondSquared = 4 * Math.PI; // TODO: tune

        public static final double xTolerance = 0.05;
        public static final double yTolerance = 0.05;
        public static final double rotTolerance = Units.degreesToRadians(5);
    }

    public static class ShooterConstants {
        public static final double hoodConversionRotToDeg = 360 / 129.6;
        public static final double MIN_RPS = 0;
        public static final double MAX_RPS = 100;

        /*
         * Tuned shot tables (distance to hub in meters -> value).
         * These are the single source of truth for shot parameters; they were
         * calibrated with the robot stationary.
         */
        public static final InterpolatingDoubleTreeMap RPS_MAP = new InterpolatingDoubleTreeMap();
        static {
            RPS_MAP.put(2.0, 47d);
            RPS_MAP.put(3.0, 50d);
            RPS_MAP.put(4.0, 52d);
            RPS_MAP.put(5.0, 55d);
            RPS_MAP.put(6.0, 62d);
            RPS_MAP.put(10.0, 85d);
        }

        public static final InterpolatingDoubleTreeMap HOOD_MAP = new InterpolatingDoubleTreeMap();
        static {
            HOOD_MAP.put(2.0, 0d);
            HOOD_MAP.put(3.0, 2.5d);
            HOOD_MAP.put(4.0, 5d);
            HOOD_MAP.put(5.0, 5.5d);
            HOOD_MAP.put(6.0, 6.5d);
            HOOD_MAP.put(10.0, 8d);
        }

        // Time of flight (seconds) by distance. Feeds the SOTF virtual-target
        // lead (v * tof), so errors here directly under/over-correct moving
        // shots. TODO: validate against real footage — the ball-physics model
        // in the sim visualization suggests real flight times may be ~3-4x
        // longer than these values.
        public static final InterpolatingDoubleTreeMap TOF_MAP = new InterpolatingDoubleTreeMap();
        static {
            TOF_MAP.put(2.0, 0.2);
            TOF_MAP.put(3.0, 0.25);
            TOF_MAP.put(4.0, 0.3);
            TOF_MAP.put(5.0, 0.35);
            TOF_MAP.put(6.0, 0.4);
            TOF_MAP.put(10.0, 0.7);
        }

        /*
         * Passing shot tables — NOT YET TUNED (all zeros). ShotCalc falls back
         * to the hub tables until these contain real data.
         */
        public static final InterpolatingDoubleTreeMap PASSING_TOF_MAP = new InterpolatingDoubleTreeMap();
        public static final InterpolatingDoubleTreeMap PASSING_HOOD_MAP = new InterpolatingDoubleTreeMap();
        public static final InterpolatingDoubleTreeMap PASSING_RPS_MAP = new InterpolatingDoubleTreeMap();
        static {
            for (double d = 2.0; d <= 6.0; d += 0.5) {
                PASSING_TOF_MAP.put(d, 0d);
                PASSING_HOOD_MAP.put(d, 0d);
                PASSING_RPS_MAP.put(d, 0d);
            }
        }

        /*
         * Historical polynomial-regression tuning data, superseded by the maps
         * above. Kept for reference:
         * hood:    (2,0), (3,2.5), (4,5), (5,5.5), (6,6.5), (10,8)     [deg 2]
         * rps:     (2,47), (3,50), (4,50), (5,55), (6,62), (10,85)     [deg 2]
         * tof:     (2,0.25), (3,0.30), (4,0.35), (5,0.40), (6,0.45)    [deg 2]
         */

        public static final double activeWaitingSpeed = 30;
        public static final double inactiveWaitingSpeed = 0;

        public static final double shooterMotionMagicExpoK_V = 0.1;
        public static final double shooterMotionMagicExpoK_A = 0.1;
        public static final double shooterMotionMagicAccel = 400; // rps^2
        public static final double shooterMotionMagicJerk = 4000; // rps^2/s

        public static final int SupplyCurrentLimit = 80;
        public static final int StatorCurrentLimit = 80;

        public static final int hoodSupplyCurrentLimit = 15;
        public static final int hoodStatorCurrentLimit = 15;

        // Supply current (amps) above which hood homing considers itself
        // against the hard stop. 0 disables auto-zeroing during HOMING
        // (the state will hold gently against the stop but never re-zero).
        public static final double homingThreshold = 0; // TODO: tune (amps)

        // Flywheel is "ready" when actual speed is within this many RPS of
        // the setpoint.
        public static final double readyToleranceRPS = 3.5;

        public static final int hoodMotorID = 53;
        public static final int shooterMotor1ID = 61;
        public static final int shooterMotor2ID = 60;

        public static final double[] hoodPID = { 2.0, 0, 0 };
        public static final double[] hoodSVA = { 0.0, 0, 0 };

        public static final double[] shooterPID = { 1.5, 0, 0.02 };
        public static final double[] shooterSVA = { 0.5, 0.117, 0.02 };

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
        public static final double intakeMotionMagicExpoK_V = 0.13;
        public static final double intakeMotionMagicExpoK_A = 0.1;

        public static final int SupplyCurrentLimit = 40;
        public static final int StatorCurrentLimit = 100;

        public static final int ExtensionSupplyCurrentLimit = 80;
        public static final int ExtensionStatorCurrentLimit = 50;

        public static final int intakeMotorID = 32;
        public static final int intakeExtensionMotorID = 31;
        public static final int canRangeID = 33;

        public static final double intakingPosition = 10;
        public static final double intakingSpeed = -0.9;
        // Slower Motion Magic kA applied to the extension while SCORING
        // (slow squeeze), restored on leaving the state.
        public static final double slowerIntakeKa = 1.0;
        // CANrange distance (meters) beyond which the extension is considered
        // at home, used only by the (currently unbound) RESET auto-zero state.
        // 0 disables it. The intended re-zero workflow after a big impact is
        // manual: D-pad nudge the intake into the hard stop, then press the
        // driver's back button (setZero).
        public static final double intakeExtensionHomingThreshold = 0;
        public static final double retractingPos = 0;
        // Duty cycle used for manual nudging and homing crawl.
        public static final double manualDutyCycle = 0.01;

        public static final double[] intakePID = { 0.3, 0, 0 };
        public static final double[] intakeSVA = { 0, 0.13, 0.01 };

        public enum IntakeWantedState {
            IDLE,
            INTAKE,
            RETRACT,
            RESET,
            SCORE,
            OUTTAKE,
            MANUAL_CONTROL_POS,
            MANUAL_CONTROL_NEG,
            MANUAL_IDLE,
            MANUAL_RESET
        }

        public enum SystemState {
            IDLING,
            INTAKING,
            RETRACTING,
            RESETING,
            SCORING,
            OUTTAKING,
            IN_MANUAL_CONTROL_POS,
            IN_MANUAL_CONTROL_NEG,
            IN_MANUAL_IDLE,
            IN_MANUAL_RESET
        }
    }

    public static class TurretConstants {
        public static final int SupplyCurrentLimit = 40;
        public static final int StatorCurrentLimit = 120;

        public static final int turretMotorID = 50;
        public static final int encoderID = 54;

        public static final double passAimPosition = 0;
        public static final double hubPresetPosition = 0;
        public static final double trenchPresetPositionL = .53;
        public static final double trenchPresetPositionR = .50;

        // Mechanism rotations. 0.05 rot = 18 degrees — generous on purpose
        // while SOTF is being tuned; tighten toward 0.007 once trusted.
        public static final double tolerance = 0.05;

        // Soft limits (mechanism rotations) and gearing.
        public static final double ccwLimit = 0.85;
        public static final double cwLimit = -0.85;
        public static final double gearRatio = 38.8888888889;

        public static final double[] turretPID = { 51, 0, 0 };
        public static final double[] turretSVA = { 0, 0, 0 };

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
        public static final int SupplyCurrentLimit = 40;
        public static final int StatorCurrentLimit = 100;

        public static final int towerMotorID = 41;
        public static final int spindexerMotorID = 40;
        public static final int rollerMotorID = 42;

        public static final double feederIntakeSpeed = 0;
        public static final double feederShootSpeed = 0.8;
        public static final double feederReverseSpeed = -0.7;

        // While PASSING, hold fire inside this field-Y band (the hub shadow)
        // so passes don't hit the hub structure.
        public static final double passExclusionYMin = 3.53;
        public static final double passExclusionYMax = 4.53;

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
        // 60 LEDs per 1m strip [spacing = 1m / #ofLEDs]
        public static final Distance spacing = Meters.of(1.0 / 60.0);
        // LED Strip
        public static final int led_port = 0;
        public static final int led_length = 40; // 48 LEDs, 24 a side
        public static final int led_brightness = 30;
        // Signal LED sector (on shooter)
        public static final int signal_length = 10; // 10, 5 a side

        public enum LightsType {
            ENDGAME,
            CLIMB,
            SHOOTING,
            INTAKE,
            IDLE,
            DISABLED
        }

        // RGB Color Map
        public static final Map<String, Color> RGBColors = Map.ofEntries(
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

        // RBG Color Map (new LEDs)
        public static final Map<String, Color> RBGColors = Map.ofEntries(
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

        // GRB Color Map (old LEDs)
        public static final Map<String, Color> GRBColors = Map.ofEntries(
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
    }

    public static class VisionConstants {
        public static final Transform2d turretToCenter = new Transform2d(
                Units.inchesToMeters(-6.5),
                Units.inchesToMeters(-6),
                new Rotation2d());

        public static final Transform3d kRobotToCam = new Transform3d(
                new Translation3d(
                        Units.inchesToMeters(6.643),
                        -Units.inchesToMeters(0.616),
                        Units.inchesToMeters(17.467 + 2.7525)),
                new Rotation3d(
                        0,
                        Units.degreesToRadians(20),
                        0));
        // center back cam
        public static final Transform3d kRobotToCam2 = new Transform3d(
                new Translation3d(
                        -Units.inchesToMeters(12.844),
                        Units.inchesToMeters(0.848),
                        Units.inchesToMeters(12.195)),
                new Rotation3d(
                        0,
                        Units.degreesToRadians(20),
                        Units.degreesToRadians(135)));
        // corner camera
        public static final Transform3d kRobotToCam3 = new Transform3d(
                new Translation3d(
                        -Units.inchesToMeters(12.843),
                        -Units.inchesToMeters(12.851),
                        Units.inchesToMeters(12.195)),
                new Rotation3d(
                        0,
                        Units.degreesToRadians(20),
                        Units.degreesToRadians(225)));

        public static final String cameraName = "camera1";
        public static final String camera2Name = "camera2";
        public static final String camera3Name = "camera3";

        /* standard deviations for vision calculations */
        public static final edu.wpi.first.math.Vector<N3> kSingleTagStdDevs = VecBuilder.fill(4, 4, 4);
        public static final edu.wpi.first.math.Vector<N3> kMultiTagStdDevs = VecBuilder.fill(4, 4, 4);
        public static final edu.wpi.first.math.Vector<N3> odoStdDEvs = VecBuilder.fill(.2, .2, .05);
        public static final double odometryUpdateFrequency = 250;
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

        public static final Translation2d BLUE_HUB_POSE = new Translation2d(4.62, 4.03);
        public static final Translation2d RED_HUB_POSE = new Translation2d(12, 4.03);

        public static final Translation2d BLUE_PASS_SPOT_1 = new Translation2d(1, 3);
        public static final Translation2d BLUE_PASS_SPOT_2 = new Translation2d(1, 5);
        public static final Translation2d RED_PASS_SPOT_1 = new Translation2d(14.5, 3);
        public static final Translation2d RED_PASS_SPOT_2 = new Translation2d(14.5, 5);

        public static final Map<ScoringZone, Pose2d> scoringZoneLUT = Map.ofEntries(
                Map.entry(ScoringZone.RED_PASSING_1, new Pose2d(RED_PASS_SPOT_1, new Rotation2d())),
                Map.entry(ScoringZone.RED_PASSING_2, new Pose2d(RED_PASS_SPOT_2, new Rotation2d())),
                Map.entry(ScoringZone.RED_HUB, new Pose2d(RED_HUB_POSE, new Rotation2d())),
                Map.entry(ScoringZone.BLUE_PASSING_1, new Pose2d(BLUE_PASS_SPOT_1, new Rotation2d())),
                Map.entry(ScoringZone.BLUE_PASSING_2, new Pose2d(BLUE_PASS_SPOT_2, new Rotation2d())),
                Map.entry(ScoringZone.BLUE_HUB, new Pose2d(BLUE_HUB_POSE, new Rotation2d())),
                Map.entry(ScoringZone.NO_TRACK, Pose2d.kZero));
    }
}
