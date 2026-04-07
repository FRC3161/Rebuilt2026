package frc.util.SOTM;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.FieldConstants;
import frc.robot.Constants.FieldConstants.ScoringZone;
import frc.robot.subsystems.Drive.CommandSwerveDrivetrain;
import frc.robot.subsystems.Scoring.Turret;

// import com.team254.lib.geometry.Pose2d;
// import com.team254.lib.geometry.Rotation2d;

public class AimingParameters {
    // private final double range;
    private final Turret turret;
    private final CommandSwerveDrivetrain drive;
    // private Transform2d field_to_vehicle;
    // private Transform2d field_to_goal;
    // private Rotation2d robot_to_goal_rotation;

    public AimingParameters(
            Turret m_turret,
            CommandSwerveDrivetrain m_drivetrain) {
        this.drive = m_drivetrain;
        this.turret = m_turret;
        // this.field_to_vehicle = field_to_vehicle;
        // this.field_to_goal = field_to_goal;
        // final Transform2d vehicle_to_goal =
        // field_to_vehicle.inverse().plus(field_to_goal);
        // this.range = vehicle_to_goal.getTranslation().getNorm();
        // this.robot_to_goal_rotation = vehicle_to_goal.getTranslation().getAngle();
    }

    // public boolean getIsLatest() {
    // return this.is_latest;
    // }

    public Pose2d getFieldToRobot() {
        return drive.getCurrentTurretPose();
    }

    public double getRange() {
        return drive.getDistanceFromHub();
    }

    public ScoringZone getZone() {
        Alliance alliance = DriverStation.getAlliance().orElse(Alliance.Blue);
        if (alliance.equals(Alliance.Blue)) {
            if (0 < drive.getPose().getX() && drive.getPose().getX() < 4.6) {
                return ScoringZone.BLUE_HUB;
            } else {
                if (4.03 < drive.getPose().getY()) {
                    return ScoringZone.BLUE_PASSING_2;
                } else {
                    return ScoringZone.BLUE_PASSING_1;
                }
            }
        } else {
            if (11.9 < drive.getPose().getX() && drive.getPose().getX() < 16.6) {
                return ScoringZone.RED_HUB;
            } else {
                if (4.03 < drive.getPose().getY()) {
                    return ScoringZone.RED_PASSING_2;
                } else {
                    return ScoringZone.RED_PASSING_1;
                }
            }
        }
    }

    public Pose2d getGoalLocation() {
        return FieldConstants.scoringZoneLUT.get(getGoalLocation());
    }

    // public double getRange() {
    // return ;
    // }

    public Transform2d getVehicleToGoal() {
        return getGoalLocation().minus(drive.getCurrentTurretPose());
    }

    public Rotation2d getRobotToGoalRotation() {
        return getGoalLocation().getRotation().minus(drive.getCurrentTurretPose().getRotation());
    }

}