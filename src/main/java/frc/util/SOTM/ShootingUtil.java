package frc.util.SOTM;

// import com.team254.lib.geometry.Pose2d;
// import com.team254.lib.geometry.Rotation2d;
// import com.team254.lib.geometry.Translation2d;
// import com.team254.lib.geometry.Twist2d;
// import com.team254.lib.util.Units;
// import com.team254.lib.util.Util;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.util.Units;

public class ShootingUtil {
    public static class ShootingParameters {
        public double effective_yaw_angle_deg;
        public double effective_range_m;

        public double effective_yaw_feedforward_velocity_deg_s;
        public double effective_range_feedforward_m_s;

        public double uncompensated_yaw_angle_deg;
        public double uncompensated_range_m;

        public ShootingParameters() {
        }
    }

    public static ShootingParameters getShootingParameters(
            double turret_angle_deg,
            Pose2d turretToField,
            AimingParameters aimingParams,
            double shotTime,
            Twist2d robotVelocity) {
        ShootingParameters params = new ShootingParameters();
        Rotation2d turret_error = turretToField
                .transformBy(new Transform2d(aimingParams.getGoalLocation().getTranslation(),
                        aimingParams.getGoalLocation().getRotation()))
                .getTranslation().getAngle();
        params.uncompensated_yaw_angle_deg = turret_angle_deg + turret_error.getDegrees();
        params.effective_yaw_angle_deg = params.uncompensated_yaw_angle_deg;
        Translation2d velocity_translational = new Translation2d(robotVelocity.dx, robotVelocity.dy);
        // Rotate by robot-to-goal rotation; x = radial component (positive towards
        // goal), y = tangential component (positive means turret needs negative lead).
        velocity_translational = velocity_translational.rotateBy(aimingParams.getRobotToGoalRotation().unaryMinus());

        double tangential_component = velocity_translational.getY();
        double radial_component = velocity_translational.getX();
        double angular_component = Units.radiansToDegrees(robotVelocity.dtheta);
        params.uncompensated_range_m = aimingParams.getRange();
        params.effective_range_m = params.uncompensated_range_m;

        double shot_speed = params.uncompensated_range_m / shotTime - radial_component;
        if (shot_speed < 0.0)
            shot_speed = 0.0;
        double turret_adjustment = Units.radiansToDegrees(Math.atan2(-tangential_component, shot_speed));
        params.effective_yaw_angle_deg += turret_adjustment;
        params.effective_range_m = shotTime
                * Math.sqrt(tangential_component * tangential_component + shot_speed * shot_speed);

        // TODO(revise feedforwards to include derivative of motion compensation)
        // Feedforward is (opposite of) tangential velocity about goal + angular
        // velocity in local frame.
        params.effective_yaw_feedforward_velocity_deg_s = -(angular_component
                + Units.radiansToDegrees(tangential_component / params.uncompensated_range_m));
        // Feedforward is (opposite of) radial velocity in local frame.
        params.effective_range_feedforward_m_s = -radial_component;
        return params;
    }
}