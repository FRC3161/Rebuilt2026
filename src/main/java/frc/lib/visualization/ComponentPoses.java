package frc.lib.visualization;

import java.util.ArrayList;
import java.util.List;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.mechanisms.Mechanism;

/**
 * Publishes where each moving part of the robot is, as a Pose3d[] that
 * AdvantageScope uses to move the parts of a custom robot model (its
 * "Components"). Every part's model is exported in its CAD position, so a
 * part's pose is just how far it has moved from there: a rotation about its
 * pivot, or a slide along its direction. Parts riding on another part (an arm
 * on an elevator) add their motion on top of their parent's.
 *
 * All points and directions are in the robot frame: meters from the robot's
 * center on the floor, X forward, Y left, Z up.
 */
public class ComponentPoses extends SubsystemBase {
    /** For parts that sit directly on the robot. */
    public static final int ROBOT = -1;

    private record Part(int parent, Supplier<Pose3d> motion) {
    }

    private final List<Part> parts = new ArrayList<>();
    private final StructArrayPublisher<Pose3d> publisher;
    private Pose3d[] poses = new Pose3d[0];

    /** @param topic NetworkTables topic for the Pose3d[] (drag it onto the robot in AdvantageScope). */
    public ComponentPoses(String topic) {
        publisher = NetworkTableInstance.getDefault().getStructArrayTopic(topic, Pose3d.struct).publish();
    }

    /**
     * A part that turns about an axis (pivot, turret).
     *
     * @param parent      index of the part it rides on, or ROBOT
     * @param rotations   its position in rotations
     * @param cadRotations its position in the CAD model
     * @param point       a point on the axis
     * @param axis        the axis direction; positive rotations turn counterclockwise looking down it
     * @return this part's index (its model is model_INDEX.glb)
     */
    public int pivot(int parent, DoubleSupplier rotations, double cadRotations, Translation3d point, Translation3d axis) {
        var unitAxis = VecBuilder.fill(axis.getX(), axis.getY(), axis.getZ());
        return add(parent, () -> {
            double radians = Units.rotationsToRadians(rotations.getAsDouble() - cadRotations);
            Rotation3d turn = new Rotation3d(unitAxis, radians);
            return new Pose3d(point.minus(point.rotateBy(turn)), turn);
        });
    }

    /**
     * A part that slides (elevator, linear intake).
     *
     * @param meters    its position in meters
     * @param cadMeters its position in the CAD model
     * @param direction unit direction it moves for positive meters
     */
    public int slide(int parent, DoubleSupplier meters, double cadMeters, Translation3d direction) {
        return add(parent, () -> new Pose3d(direction.times(meters.getAsDouble() - cadMeters), new Rotation3d()));
    }

    /** A part that moves with its parent but not on its own (or isn't animated yet). */
    public int fixed(int parent) {
        return add(parent, Pose3d::new);
    }

    private int add(int parent, Supplier<Pose3d> motion) {
        if (parent >= parts.size()) throw new IllegalArgumentException("Add a part's parent before the part");
        parts.add(new Part(parent, motion));
        return parts.size() - 1;
    }

    /** The position of a library mechanism by name ("Subsystem/Part"); 0 until it exists. */
    public static DoubleSupplier mechanism(String name) {
        return () -> {
            Mechanism m = Mechanism.find(name);
            return m == null ? 0.0 : m.getPosition();
        };
    }

    /** The latest poses, in part order. */
    public Pose3d[] getPoses() {
        return poses;
    }

    /** Stops publishing (tests). */
    public void close() {
        CommandScheduler.getInstance().unregisterSubsystem(this);
        publisher.close();
    }

    @Override
    public void periodic() {
        Pose3d[] next = new Pose3d[parts.size()];
        for (int i = 0; i < parts.size(); i++) {
            Part part = parts.get(i);
            Pose3d own = part.motion().get();
            next[i] = part.parent() == ROBOT
                    ? own
                    : next[part.parent()].transformBy(new Transform3d(own.getTranslation(), own.getRotation()));
        }
        poses = next;
        publisher.set(next);
    }
}
