package frc.robot.commands.Drive;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.AutoConstants;
import frc.robot.subsystems.Drive.CommandSwerveDrivetrain;

public class DriveToLocation {

    /**
     * Pathfinds to the target pose, ending early once the robot is within
     * tolerance. The error is re-evaluated every loop (inside the lambda) and
     * compared with absolute values — evaluating it once at construction
     * made the end condition a constant.
     */
    public static Command pathFindTo(Pose2d target, CommandSwerveDrivetrain drive) {
        Command command = AutoBuilder.pathfindToPose(target,
                new PathConstraints(
                        AutoConstants.kMaxSpeedMetersPerSecond,
                        AutoConstants.kMaxAccelerationMetersPerSecondSquared,
                        AutoConstants.kMaxAngularSpeedRadiansPerSecond,
                        AutoConstants.kMaxAngularSpeedRadiansPerSecondSquared),
                0);
        return command.until(() -> {
            Transform2d error = target.minus(drive.getPose());
            return Math.abs(error.getX()) < AutoConstants.xTolerance
                    && Math.abs(error.getY()) < AutoConstants.yTolerance
                    && Math.abs(error.getRotation().getRadians()) < AutoConstants.rotTolerance;
        });
    }
}
