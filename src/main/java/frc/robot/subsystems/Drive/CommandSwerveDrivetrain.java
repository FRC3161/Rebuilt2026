package frc.robot.subsystems.Drive;

import static edu.wpi.first.units.Units.*;

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
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Robot;
import frc.robot.Constants.FieldConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.Constants.VisionConstants;
import frc.robot.subsystems.Drive.TunerConstants.TunerSwerveDrivetrain;
import frc.robot.subsystems.Scoring.ShotCalc;

public class CommandSwerveDrivetrain extends TunerSwerveDrivetrain implements Subsystem {
    public final Vision vision = new Vision();
    public final SlipDetection slipDetection = new SlipDetection();
    private final Field2d TheField = new Field2d();

    private static final double kSimLoopPeriod = 0.004;
    private Notifier m_simNotifier = null;
    private double m_lastSimTime;

    private final SwerveRequest.ApplyRobotSpeeds m_pathApplyRobotSpeeds = new SwerveRequest.ApplyRobotSpeeds();

    private static final Rotation2d kBlueAlliancePerspectiveRotation = Rotation2d.kZero;
    private static final Rotation2d kRedAlliancePerspectiveRotation = Rotation2d.k180deg;
    private boolean m_hasAppliedOperatorPerspective = false;

    /*
     * Latest shoot-on-the-fly solution, recomputed every periodic. Consumers
     * (Shooter, Turret) read this rather than re-running the solver.
     * shotCommandTimestamp lets the turret compensate for the loop delay
     * between solve and apply.
     */
    public ShotCalc.ShooterCommand currentShotCommand = new ShotCalc.ShooterCommand(0, new Rotation2d(), 0);
    public double shotCommandTimestamp = 0;

    /* ---------------- Ball trajectory visualization (sim only) -----------
     * Draws the shot the robot is COMMANDED to take (currentShotCommand),
     * including velocity inherited from the chassis, so a SOTF math error
     * shows up as a visible miss. TODO: calibrate the exit-speed model at
     * standstill so table shots land in the hub, then trust moving shots. */
    private static final double kWheelDiameterMeters = Units.inchesToMeters(4.0);
    private static final double kExitEfficiency = 0.45; // TODO: calibrate
    private static final double kHoodHomeAngleDeg = 18.0; // TODO: verify
    private static final double kLaunchHeightMeters = 0.46; // TODO: verify

    private final StructArrayPublisher<Pose3d> trajectoryPublisher = NetworkTableInstance.getDefault()
            .getTable("SmartDashboard")
            .getStructArrayTopic("ball trajectory 3d", Pose3d.struct)
            .publish();

    public Pose2d getPose() {
        return this.getState().Pose;
    }

    /** Field-relative chassis speeds. */
    public ChassisSpeeds getFieldSpeeds() {
        return ChassisSpeeds.fromRobotRelativeSpeeds(getState().Speeds, getPose().getRotation());
    }

    /**
     * Current target: the hub while inside our scoring zone, otherwise a
     * passing spot on the same side of the field the robot is currently on
     * (a same-side pass is shorter and safer than throwing cross-field).
     */
    public Translation2d getScoringLocation() {
        Alliance alliance = DriverStation.getAlliance().orElse(Alliance.Blue);
        double x = getPose().getX();
        double y = getPose().getY();
        if (alliance == Alliance.Blue) {
            if (0 < x && x < 4.6) {
                return FieldConstants.BLUE_HUB_POSE;
            }
            return y > 4.03 ? FieldConstants.BLUE_PASS_SPOT_2 : FieldConstants.BLUE_PASS_SPOT_1;
        } else {
            if (11.9 < x && x < 16.6) {
                return FieldConstants.RED_HUB_POSE;
            }
            return y > 4.03 ? FieldConstants.RED_PASS_SPOT_2 : FieldConstants.RED_PASS_SPOT_1;
        }
    }

    /** Our alliance's hub position. */
    public Translation2d getHub() {
        return DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red
                ? FieldConstants.RED_HUB_POSE
                : FieldConstants.BLUE_HUB_POSE;
    }

    /**
     * Field pose of the turret. The turret is mounted offset from robot
     * center and faces backwards relative to the chassis (hence the 180
     * degree rotation).
     */
    public Pose2d getCurrentTurretPose() {
        Pose2d turretPose = getPose().transformBy(VisionConstants.turretToCenter);
        return turretPose.rotateAround(turretPose.getTranslation(), Rotation2d.k180deg);
    }

    public double getDistanceFromHub() {
        return getHub().getDistance(getCurrentTurretPose().getTranslation());
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

    /* ---------------- SysId ---------------- */
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
        SmartDashboard.putData("DRIVE/The field", TheField);
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
        slipDetection.update(this);

        if (DriverStation.getAlliance().isPresent() || Robot.isSimulation()) {
            shotCommandTimestamp = Timer.getFPGATimestamp();
            currentShotCommand = ShotCalc.calculateSOTF(
                    getPose().getTranslation(),
                    getCurrentTurretPose().getTranslation(),
                    getFieldSpeeds(),
                    getScoringLocation());
        }

        if (Robot.isSimulation()) {
            SmartDashboard.putNumberArray("SIM/SimRobotPose", new double[] {
                    getPose().getX(),
                    getPose().getY(),
                    getPose().getRotation().getRadians()
            });
            updateBallTrajectory();
        }

        // Field visualization
        TheField.getObject("robot").setPose(getPose());
        TheField.getObject("target").setPose(new Pose2d(getScoringLocation(), new Rotation2d()));
        TheField.getObject("turret").setPose(getCurrentTurretPose());

        ChassisSpeeds fieldSpeeds = getFieldSpeeds();
        SmartDashboard.putNumber("DRIVE/Pose X", getPose().getX());
        SmartDashboard.putNumber("DRIVE/Pose Y", getPose().getY());
        SmartDashboard.putNumber("DRIVE/Pose Rotation", getPose().getRotation().getDegrees());
        SmartDashboard.putNumber("DRIVE/Distance From Hub", getDistanceFromHub());
        SmartDashboard.putNumber("DRIVE/Field Vx", fieldSpeeds.vxMetersPerSecond);
        SmartDashboard.putNumber("DRIVE/Field Vy", fieldSpeeds.vyMetersPerSecond);
        SmartDashboard.putNumber("DRIVE/Omega Rad", fieldSpeeds.omegaRadiansPerSecond);

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

    /**
     * Publishes the 3D arc of the shot the robot is currently commanded to
     * take. Launch direction and speed come from currentShotCommand; the
     * chassis velocity (translation + turret tangential velocity) is added
     * to the ball, exactly as it is in real life. Also publishes the ball's
     * height when it crosses the target's floor position, so accuracy can be
     * judged numerically instead of by eye.
     */
    private void updateBallTrajectory() {
        ShotCalc.ShooterCommand cmd = currentShotCommand;
        if (cmd.RPS() <= 0) {
            trajectoryPublisher.set(new Pose3d[0]);
            return;
        }

        Translation2d robotPos = getPose().getTranslation();
        Translation2d turretPos = getCurrentTurretPose().getTranslation();

        // Exit velocity model — the calibration targets
        double hoodDeg = cmd.hoodAngle() * ShooterConstants.hoodConversionRotToDeg;
        double launchAngleRad = Math.toRadians(90.0 - (kHoodHomeAngleDeg + hoodDeg));
        double exitSpeed = cmd.RPS() * Math.PI * kWheelDiameterMeters * kExitEfficiency;
        double horizontalSpeed = exitSpeed * Math.cos(launchAngleRad);
        double verticalSpeed = exitSpeed * Math.sin(launchAngleRad);

        // Ball's field velocity = launch velocity + inherited chassis velocity
        ChassisSpeeds fieldSpeeds = getFieldSpeeds();
        double omega = fieldSpeeds.omegaRadiansPerSecond;
        Translation2d turretOffset = turretPos.minus(robotPos);
        Translation2d inheritedVelocity = new Translation2d(
                fieldSpeeds.vxMetersPerSecond,
                fieldSpeeds.vyMetersPerSecond)
                .plus(new Translation2d(-omega * turretOffset.getY(), omega * turretOffset.getX()));

        double vx = cmd.turretAngle().getCos() * horizontalSpeed + inheritedVelocity.getX();
        double vy = cmd.turretAngle().getSin() * horizontalSpeed + inheritedVelocity.getY();

        double g = 9.81;
        double timeToGround = (verticalSpeed
                + Math.sqrt(verticalSpeed * verticalSpeed + 2 * g * kLaunchHeightMeters)) / g;

        int steps = 20;
        Pose3d[] arc = new Pose3d[steps + 1];
        for (int i = 0; i <= steps; i++) {
            double t = (timeToGround / steps) * i;
            arc[i] = new Pose3d(
                    turretPos.getX() + vx * t,
                    turretPos.getY() + vy * t,
                    Math.max(0, kLaunchHeightMeters + verticalSpeed * t - 0.5 * g * t * t),
                    new Rotation3d());
        }
        trajectoryPublisher.set(arc);

        // Height when the ball crosses the target's ground position — compare
        // against the hub opening height to judge the shot numerically.
        Translation2d goal = getScoringLocation();
        double groundSpeed = Math.hypot(vx, vy);
        if (groundSpeed > 0.01) {
            double tAtGoal = goal.getDistance(turretPos) / groundSpeed;
            double heightAtGoal = kLaunchHeightMeters + verticalSpeed * tAtGoal - 0.5 * g * tAtGoal * tAtGoal;
            SmartDashboard.putNumber("SIM/Ball Height At Target", heightAtGoal);
            SmartDashboard.putNumber("SIM/Ball Flight Time To Target", tAtGoal);
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
