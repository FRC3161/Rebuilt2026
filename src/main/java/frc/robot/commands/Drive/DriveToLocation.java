package frc.robot.commands.Drive;

import java.util.Set;
import java.util.function.Supplier;

import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.DeferredCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Constants.AutoConstants;
import frc.robot.subsystems.Drive.CommandSwerveDrivetrain;

public class DriveToLocation {
    // public PIDController xController = new PIDController(0, 0, 0);
    // public PIDController yController = new PIDController(0, 0, 0);
    // public PIDController rotController = new PIDController(0, 0, 0);

    // option 1
    public static Command pathFindTo(Pose2d target, CommandSwerveDrivetrain drive) {
        var command = AutoBuilder.pathfindToPose(target,
                new PathConstraints(
                        AutoConstants.kMaxSpeedMetersPerSecond,
                        AutoConstants.kMaxAccelerationMetersPerSecondSquared,
                        AutoConstants.kMaxAngularSpeedRadiansPerSecond,
                        AutoConstants.kMaxAngularSpeedRadiansPerSecondSquared),
                0);
        command.addRequirements();
        Transform2d error = target.minus(drive.getPose());
        return command.until(
                () -> error.getX() < AutoConstants.xTolerance &&
                        error.getY() < AutoConstants.yTolerance &&
                        error.getRotation().getRadians() < AutoConstants.rotTolerance);
    }
    // option 2
    // public Command driveTo(Pose2d target, CommandSwerveDrivetrain drive) {
    // rotController.enableContinuousInput(-Math.PI, Math.PI);
    // double xOutput = xController.calculate(drive.getPose().getX(),
    // target.getX());
    // double yOutput = yController.calculate(drive.getPose().getY(),
    // target.getY());
    // double rotOutput =
    // rotController.calculate(drive.getPose().getRotation().getRadians(),
    // target.getRotation().getRadians());

    // xOutput = MathUtil.clamp(xOutput, -AutoConstants.kMaxSpeedMetersPerSecond,
    // AutoConstants.kMaxSpeedMetersPerSecond);
    // yOutput = MathUtil.clamp(yOutput, -AutoConstants.kMaxSpeedMetersPerSecond,
    // AutoConstants.kMaxSpeedMetersPerSecond);
    // rotOutput = MathUtil.clamp(rotOutput,
    // -AutoConstants.kMaxAngularSpeedRadiansPerSecond,
    // AutoConstants.kMaxAngularSpeedRadiansPerSecond);

    // final SwerveRequest.FieldCentric align = new SwerveRequest.FieldCentric();
    // xController.setTolerance(0.05);
    // yController.setTolerance(0.05);
    // rotController.setTolerance(Math.toRadians(2));
    // return
    // drive.applyRequest(() -> {
    // return align
    // .withVelocityX(xOutput)
    // .withVelocityY(yOutput)
    // .withRotationalRate(rotOutput);
    // }).until(() ->
    // xController.atSetpoint() &&
    // yController.atSetpoint() &&
    // rotController.atSetpoint());
    // }

}