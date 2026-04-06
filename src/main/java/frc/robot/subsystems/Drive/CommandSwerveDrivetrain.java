package frc.robot.subsystems.Drive;

import static edu.wpi.first.units.Units.*;

import java.util.ArrayList;
import java.util.List;
import java.util.Optional;
import java.util.function.Supplier;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants.FieldConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.Constants.VisionConstants;
import frc.robot.Robot;
import frc.robot.subsystems.Drive.TunerConstants.TunerSwerveDrivetrain;
import frc.robot.subsystems.Scoring.ShotCalc;
import edu.wpi.first.wpilibj.Timer;

public class CommandSwerveDrivetrain extends TunerSwerveDrivetrain implements Subsystem {
    public Vision vision = new Vision();
    private Alliance allianceColor = DriverStation.getAlliance().orElse(Alliance.Blue);
    private Field2d TheField = new Field2d();
    private static final double kSimLoopPeriod = 0.004;
    private Notifier m_simNotifier = null;
    private double m_lastSimTime;

    private final SwerveRequest.ApplyFieldSpeeds m_pathApplyFieldSpeeds = new SwerveRequest.ApplyFieldSpeeds();
    private final SwerveRequest.ApplyRobotSpeeds m_pathApplyRobotSpeeds = new SwerveRequest.ApplyRobotSpeeds();

    private static final Rotation2d kBlueAlliancePerspectiveRotation = Rotation2d.kZero;
    private static final Rotation2d kRedAlliancePerspectiveRotation = Rotation2d.k180deg;
    private boolean m_hasAppliedOperatorPerspective = false;

    private final StructArrayPublisher<Pose3d> trajectoryPublisher = NetworkTableInstance.getDefault()
            .getTable("SmartDashboard")
            .getStructArrayTopic("ball trajectory 3d", Pose3d.struct)
            .publish();

    public ShotCalc.ShooterCommand currentShotCommand = new ShotCalc.ShooterCommand(0, new Rotation2d(), 0);

    public double shotCommandTimestamp = 0;

    private ChassisSpeeds filteredFieldSpeeds = new ChassisSpeeds(0, 0, 0);

    public Pose2d getPose() {
        return this.getState().Pose;
    }

    public Translation2d getScoringLocation() {
        Alliance alliance = DriverStation.getAlliance().orElse(Alliance.Blue);
        if (alliance.equals(Alliance.Blue)) {
            if (getPose().getX() < 4.6) {
                return FieldConstants.BLUE_HUB_POSE;
            } else {
                if (4.03 < getPose().getY()) {
                    return FieldConstants.BLUE_PASS_SPOT_2;
                } else {
                    return FieldConstants.BLUE_PASS_SPOT_1;
                }
            }
        } else {
            if (11.9 < getPose().getX()) {
                return FieldConstants.RED_HUB_POSE;
            } else {
                if (4.03 < getPose().getY()) {
                    return FieldConstants.RED_PASS_SPOT_2;
                } else {
                    return FieldConstants.RED_PASS_SPOT_1;
                }
            }
        }
    }

    public Pose2d getCurrentTurretPose() {
        Pose2d turretPose = getPose().transformBy(VisionConstants.turretToCenter);
        turretPose = turretPose.rotateAround(turretPose.getTranslation(), Rotation2d.k180deg);
        return turretPose;
    }

    public Translation2d getHub() {
        Translation2d goalLocation = DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red
                ? FieldConstants.RED_HUB_POSE
                : FieldConstants.BLUE_HUB_POSE;
        return goalLocation;
    }

    public double getDistanceFromHub() {
        double distance = Math.hypot(getXfromHub(), getYfromHub());
        return distance;
    }

    public double getXfromLocation(Pose2d target) {
        Alliance allianceColor = DriverStation.getAlliance().orElse(Alliance.Blue);
        if (allianceColor == Alliance.Red) {
            return Math.abs(target.getX() - getCurrentTurretPose().getX());
        } else {
            return Math.abs(target.getX() - getCurrentTurretPose().getX());
        }
    }

    public double getYfromLocation(Pose2d target) {
        Alliance allianceColor = DriverStation.getAlliance().orElse(Alliance.Blue);
        if (allianceColor == Alliance.Red) {
            return Math.abs(target.getY() - getCurrentTurretPose().getY());
        } else {
            return Math.abs(target.getY() - getCurrentTurretPose().getY());
        }
    }

    public double getXfromHub() {
        Alliance allianceColor = DriverStation.getAlliance().orElse(Alliance.Blue);
        if (allianceColor == Alliance.Red) {
            return FieldConstants.RED_HUB_POSE.getX() - getCurrentTurretPose().getX();
        } else {
            return FieldConstants.BLUE_HUB_POSE.getX() - getCurrentTurretPose().getX();
        }
    }

    public double getYfromHub() {
        Alliance allianceColor = DriverStation.getAlliance().orElse(Alliance.Blue);
        if (allianceColor == Alliance.Red) {
            return FieldConstants.RED_HUB_POSE.getY() - getCurrentTurretPose().getY();
        } else {
            return FieldConstants.BLUE_HUB_POSE.getY() - getCurrentTurretPose().getY();
        }
    }

    private void updateBallTrajectory() {
        if (!Robot.isSimulation())
            return;

        Translation2d turretPos = getCurrentTurretPose().getTranslation();
        double distance = getDistanceFromHub();

        // Use table lookups directly with offsets
        double baselineRPS = (currentShotCommand.RPS() > 0 ? currentShotCommand.RPS()
                : ShooterConstants.RPS_MAP.get(distance)) + ShotCalc.rpsOffset;
        double hoodAngleDeg = (ShooterConstants.HOOD_MAP.get(distance) + ShotCalc.hoodOffset)
                * ShooterConstants.hoodConversionRotToDeg;
        double hoodHomeAngle = 18.0;
        double launchAngleRad = Math.toRadians(90.0 - (hoodHomeAngle + hoodAngleDeg));

        double flywheelSurfaceSpeed = baselineRPS * Math.PI * (4 * 0.0254);
        double exitSpeed = flywheelSurfaceSpeed * 0.45;
        double horizontalSpeed = exitSpeed * Math.cos(launchAngleRad);
        double verticalSpeed = exitSpeed * Math.sin(launchAngleRad);

        // Use actual shot angle from ShotCalc
        double shotAngleRad = currentShotCommand.turretAngle().getRadians();

        // Shooter imparted velocity
        double shooterVx = horizontalSpeed * Math.cos(shotAngleRad);
        double shooterVy = horizontalSpeed * Math.sin(shotAngleRad);

        // Add robot velocity — ball inherits this at release
        ChassisSpeeds fieldSpeeds = ChassisSpeeds.fromRobotRelativeSpeeds(
                getState().Speeds, getPose().getRotation());
        double vx = shooterVx + fieldSpeeds.vxMetersPerSecond;
        double vy = shooterVy + fieldSpeeds.vyMetersPerSecond;

        double launchHeight = 0.46;
        double g = 9.81;
        int steps = 20;
        double[] arr = new double[steps * 3];
        int count = 0;

        double timeToGround = (verticalSpeed + Math.sqrt(verticalSpeed * verticalSpeed + 2 * g * launchHeight)) / g;

        for (int i = 0; i < steps; i++) {
            double t = (timeToGround / steps) * i;
            double x = turretPos.getX() + vx * t;
            double y = turretPos.getY() + vy * t;
            double z = launchHeight + verticalSpeed * t - 0.5 * g * t * t;
            if (z < 0)
                break;
            arr[count * 3] = x;
            arr[count * 3 + 1] = y;
            arr[count * 3 + 2] = z;
            count++;
        }

        Pose3d[] poseArray = new Pose3d[count];
        for (int i = 0; i < count; i++) {
            poseArray[i] = new Pose3d(
                    arr[i * 3],
                    arr[i * 3 + 1],
                    arr[i * 3 + 2],
                    new Rotation3d());
        }
        trajectoryPublisher.set(poseArray);

        // Debug
        SmartDashboard.putNumber("Trajectory Vertical Speed", verticalSpeed);
        SmartDashboard.putNumber("Trajectory Distance", distance);
        SmartDashboard.putNumber("Trajectory Exit Speed", exitSpeed);
        SmartDashboard.putNumber("Trajectory Horizontal Speed", horizontalSpeed);
        SmartDashboard.putNumber("Trajectory Point Count", count);
        SmartDashboard.putNumber("Trajectory Shot Angle", Math.toDegrees(shotAngleRad));
        SmartDashboard.putNumber("Trajectory VX", vx);
        SmartDashboard.putNumber("Trajectory VY", vy);
    }

    private void configureAutoBuilder() {
        try {
            var config = RobotConfig.fromGUISettings();
            AutoBuilder.configure(
                    () -> getState().Pose,
                    this::resetPose,
                    () -> getState().Speeds,
                    (speeds, feedforwards) -> setControl(
                            m_pathApplyRobotSpeeds.withSpeeds(ChassisSpeeds.discretize(speeds, 0.020))
                                    .withWheelForceFeedforwardsX(feedforwards.robotRelativeForcesXNewtons())
                                    .withWheelForceFeedforwardsY(feedforwards.robotRelativeForcesYNewtons())),
                    new PPHolonomicDriveController(
                            new PIDConstants(10, 0, 0),
                            new PIDConstants(7, 0, 0)),
                    config,
                    () -> DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red,
                    this);
        } catch (Exception ex) {
            DriverStation.reportError("Failed to load PathPlanner config and configure AutoBuilder",
                    ex.getStackTrace());
        }
    }

    private final SwerveRequest.SysIdSwerveTranslation m_translationCharacterization = new SwerveRequest.SysIdSwerveTranslation();
    private final SwerveRequest.SysIdSwerveSteerGains m_steerCharacterization = new SwerveRequest.SysIdSwerveSteerGains();
    private final SwerveRequest.SysIdSwerveRotation m_rotationCharacterization = new SwerveRequest.SysIdSwerveRotation();

    private final SysIdRoutine m_sysIdRoutineTranslation = new SysIdRoutine(
            new SysIdRoutine.Config(
                    null,
                    Volts.of(4),
                    null,
                    state -> SignalLogger.writeString("SysIdTranslation_State", state.toString())),
            new SysIdRoutine.Mechanism(
                    output -> setControl(m_translationCharacterization.withVolts(output)),
                    null,
                    this));

    private final SysIdRoutine m_sysIdRoutineSteer = new SysIdRoutine(
            new SysIdRoutine.Config(
                    null,
                    Volts.of(7),
                    null,
                    state -> SignalLogger.writeString("SysIdSteer_State", state.toString())),
            new SysIdRoutine.Mechanism(
                    volts -> setControl(m_steerCharacterization.withVolts(volts)),
                    null,
                    this));

    private final SysIdRoutine m_sysIdRoutineRotation = new SysIdRoutine(
            new SysIdRoutine.Config(
                    Volts.of(Math.PI / 6).per(Second),
                    Volts.of(Math.PI),
                    null,
                    state -> SignalLogger.writeString("SysIdRotation_State", state.toString())),
            new SysIdRoutine.Mechanism(
                    output -> {
                        setControl(m_rotationCharacterization.withRotationalRate(output.in(Volts)));
                        SignalLogger.writeDouble("Rotational_Rate", output.in(Volts));
                    },
                    null,
                    this));

    private SysIdRoutine m_sysIdRoutineToApply = m_sysIdRoutineTranslation;

    public CommandSwerveDrivetrain(
            SwerveDrivetrainConstants drivetrainConstants,
            SwerveModuleConstants<?, ?, ?>... modules) {
        super(drivetrainConstants, modules);
        if (Utils.isSimulation()) {
            startSimThread();
        }
    }

    public CommandSwerveDrivetrain(
            SwerveDrivetrainConstants drivetrainConstants,
            double odometryUpdateFrequency,
            SwerveModuleConstants<?, ?, ?>... modules) {
        super(drivetrainConstants, odometryUpdateFrequency, modules);
        if (Utils.isSimulation()) {
            startSimThread();
        }
    }

    public CommandSwerveDrivetrain(
            SwerveDrivetrainConstants drivetrainConstants,
            double odometryUpdateFrequency,
            Matrix<N3, N1> odometryStandardDeviation,
            Matrix<N3, N1> visionStandardDeviation,
            SwerveModuleConstants<?, ?, ?>... modules) {
        super(drivetrainConstants, odometryUpdateFrequency, odometryStandardDeviation, visionStandardDeviation,
                modules);
        if (Utils.isSimulation()) {
            startSimThread();
        }
        SmartDashboard.putData("The field", TheField);
        configureAutoBuilder();
    }

    public Command applyRequest(Supplier<SwerveRequest> request) {
        return run(() -> this.setControl(request.get()));
    }

    public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
        return m_sysIdRoutineToApply.quasistatic(direction);
    }

    public Command sysIdDynamic(SysIdRoutine.Direction direction) {
        return m_sysIdRoutineToApply.dynamic(direction);
    }

    @Override
    public void periodic() {
        vision.updateVision(this);
        // TODO: get rid of like half of this
        if (DriverStation.getAlliance().isPresent()) {
            ChassisSpeeds rawFieldSpeeds = ChassisSpeeds.fromRobotRelativeSpeeds(
                    getState().Speeds,
                    getPose().getRotation());

            SmartDashboard.putNumber("Raw Robot Vx", getState().Speeds.vxMetersPerSecond);
            SmartDashboard.putNumber("Raw Robot Vy", getState().Speeds.vyMetersPerSecond);
            SmartDashboard.putNumber("Field Vx Converted", rawFieldSpeeds.vxMetersPerSecond);
            SmartDashboard.putNumber("Field Vy Converted", rawFieldSpeeds.vyMetersPerSecond);

            shotCommandTimestamp = Timer.getFPGATimestamp();
            currentShotCommand = ShotCalc.calculateSOTF(
                    getPose().getTranslation(),
                    getCurrentTurretPose().getTranslation(),
                    rawFieldSpeeds,
                    getState().Speeds.omegaRadiansPerSecond,
                    getScoringLocation());
        }

        // Publish clean 50Hz pose for simulation visualization
        if (Robot.isSimulation()) {
            SmartDashboard.putNumberArray("SimRobotPose", new double[] {
                    getPose().getX(),
                    getPose().getY(),
                    getPose().getRotation().getRadians()
            });
            updateBallTrajectory();
        }

        // Update field visualization
        TheField.getObject("robot").setPose(getPose());
        TheField.getObject("target").setPose(new Pose2d(getScoringLocation(), new Rotation2d()));
        TheField.getObject("turret").setPose(getCurrentTurretPose());

        if (DriverStation.getAlliance().isPresent()) {
            ChassisSpeeds fieldSpeeds = ChassisSpeeds.fromRobotRelativeSpeeds(
                    getState().Speeds, getPose().getRotation());
            double tof = ShooterConstants.TOF_MAP.get(getDistanceFromHub());
            TheField.getObject("virtual target").setPose(
                    new Pose2d(
                            getScoringLocation().minus(
                                    new Translation2d(
                                            fieldSpeeds.vxMetersPerSecond,
                                            fieldSpeeds.vyMetersPerSecond)
                                            .times(tof)),
                            new Rotation2d()));
        }

        // SmartDashboard logging
        SmartDashboard.putNumber("Pose X", getPose().getX());
        SmartDashboard.putNumber("Pose Y", getPose().getY());
        SmartDashboard.putNumber("Pose Rotation", getPose().getRotation().getDegrees());
        SmartDashboard.putNumber("Distance From Hub", getDistanceFromHub());
        SmartDashboard.putNumber("Field Vx", ChassisSpeeds.fromRobotRelativeSpeeds(
                getState().Speeds, getPose().getRotation()).vxMetersPerSecond);
        SmartDashboard.putNumber("Field Vy", ChassisSpeeds.fromRobotRelativeSpeeds(
                getState().Speeds, getPose().getRotation()).vyMetersPerSecond);
        SmartDashboard.putNumber("Omega Rad", getState().Speeds.omegaRadiansPerSecond);
        SmartDashboard.putNumber("Distance From Target",
                getCurrentTurretPose().getTranslation().getDistance(getScoringLocation()));

        // Operator perspective
        if (Robot.isSimulation()) {
            setOperatorPerspectiveForward(kBlueAlliancePerspectiveRotation);
            m_hasAppliedOperatorPerspective = true;
        } else if (!m_hasAppliedOperatorPerspective || DriverStation.isDisabled()) {
            DriverStation.getAlliance().ifPresent(allianceColor -> {
                setOperatorPerspectiveForward(
                        allianceColor == Alliance.Red
                                ? kRedAlliancePerspectiveRotation
                                : kBlueAlliancePerspectiveRotation);
                m_hasAppliedOperatorPerspective = true;
            });
        }
    }

    private void startSimThread() {
        m_lastSimTime = Utils.getCurrentTimeSeconds();
        m_simNotifier = new Notifier(() -> {
            final double currentTime = Utils.getCurrentTimeSeconds();
            double deltaTime = currentTime - m_lastSimTime;
            m_lastSimTime = currentTime;
            updateSimState(deltaTime, RobotController.getBatteryVoltage());
        });
        m_simNotifier.startPeriodic(kSimLoopPeriod);
    }

    @Override
    public void addVisionMeasurement(Pose2d visionRobotPoseMeters, double timestampSeconds) {
        super.addVisionMeasurement(visionRobotPoseMeters, Utils.fpgaToCurrentTime(timestampSeconds));
    }

    @Override
    public void addVisionMeasurement(
            Pose2d visionRobotPoseMeters,
            double timestampSeconds,
            Matrix<N3, N1> visionMeasurementStdDevs) {
        super.addVisionMeasurement(visionRobotPoseMeters, Utils.fpgaToCurrentTime(timestampSeconds),
                visionMeasurementStdDevs);
    }

    @Override
    public Optional<Pose2d> samplePoseAt(double timestampSeconds) {
        return super.samplePoseAt(Utils.fpgaToCurrentTime(timestampSeconds));
    }
}