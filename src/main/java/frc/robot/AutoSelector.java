package frc.robot;

import java.util.HashSet;
import java.util.List;
import java.util.Set;
import java.util.function.Supplier;

import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.util.FlippingUtil;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;

/**
 * Registry of Left/Right auto pairs confirmed Y-mirror-symmetric (verified by comparing the
 * underlying Choreo .traj starting poses against PathPlannerLib's FlippingUtil.fieldSizeY,
 * not by filename convention) plus the vision-driven side detection that picks between them.
 *
 * The robot's pose is already vision-corrected continuously during the disabled period (see
 * Vision.updateVision / CommandSwerveDrivetrain.periodic), so a family's side can be decided
 * from where the robot is sitting on the field before a match starts. Both sides' Commands
 * are built once, up front (see RobotContainer.configureAutoCommands()) — only the pick
 * between the two already-built Commands happens at auto start, so nothing gets
 * parsed/constructed at the moment autonomous begins.
 *
 * Each family collapses its two hand-authored/generated .auto files into a single dashboard
 * chooser entry (RobotContainer filters the raw per-side options out of the chooser using
 * coveredAutoNames() below) — adding an entry here is the only step needed to both stop
 * hand-mirroring a new path in Choreo/PathPlanner AND declutter the dashboard; do not add
 * a pair here without verifying the underlying .traj Y-values actually mirror around
 * FlippingUtil.fieldSizeY / 2.
 */
public final class AutoSelector {
    private AutoSelector() {}

    /** Half of FlippingUtil.fieldSizeY — the raw-field Y threshold separating the two sides. */
    public static final double FIELD_MIDLINE_Y = FlippingUtil.fieldSizeY / 2.0;

    public enum Side {
        LEFT, RIGHT
    }

    /**
     * Left/Right .auto files are authored (and their Y-mirror pairs generated) in raw,
     * blue-origin field coordinates, where high Y is Blue's left / Blue's right is low Y (see
     * AutoMirroring history in AutoSelector's class doc). PathPlannerLib's AutoBuilder
     * (configured with shouldFlipPath = alliance == Red, see CommandSwerveDrivetrain) applies
     * a 180-degree rotational flip to every auto's authored path/pose when actually running
     * on Red alliance, which inverts both X and Y -- so on Red, high raw Y is physically
     * Red's RIGHT side, the opposite of Blue. drivetrain.getPose() is always in raw,
     * unflipped field coordinates (vision/odometry never apply the alliance flip), so this
     * must un-invert the raw-Y test for Red or every Red match would pick the wrong side.
     */
    public static Side sideFromPose(Pose2d pose, Alliance alliance) {
        boolean highY = pose.getY() > FIELD_MIDLINE_Y;
        boolean onRed = alliance == Alliance.Red;
        return (highY != onRed) ? Side.LEFT : Side.RIGHT;
    }

    /**
     * One entry per mirror-paired family: the combined dashboard label, and a display name +
     * builder for each side's Command. Names are tracked separately from the Command objects
     * so the dashboard can report which concrete auto will run without depending on
     * PathPlannerAuto's internal naming. A rightName that isn't a real file on disk (e.g. one
     * ending in "(Auto-Mirrored)") just means that side is generated via mirror=true rather
     * than pointing at a hand-authored .auto file.
     */
    public record AutoFamily(
            String label,
            String leftName, Supplier<Command> leftBuilder,
            String rightName, Supplier<Command> rightBuilder) {
    }

    // Right-side pointers favor an already-validated hand-authored .auto where one exists;
    // families with no hand-authored Right file fall back to the generated mirror. Repoint
    // to a plain PathPlannerAuto(name) once a hand-authored replacement is validated and
    // checked in.
    public static final List<AutoFamily> AUTO_DETECT_FAMILIES = List.of(
            new AutoFamily("Trench A",
                    "Left Trench A", () -> new PathPlannerAuto("Left Trench A"),
                    "Right Trench A", () -> new PathPlannerAuto("Right Trench A")),
            new AutoFamily("Trench B",
                    "Left Trench B", () -> new PathPlannerAuto("Left Trench B"),
                    "Right Trench B", () -> new PathPlannerAuto("Right Trench B")),
            new AutoFamily("Theory Dance",
                    "Theory Dance L", () -> new PathPlannerAuto("Theory Dance L"),
                    "Theory Dance R", () -> new PathPlannerAuto("Theory Dance R")),
            new AutoFamily("Bump LS",
                    "LBumpLS", () -> new PathPlannerAuto("LBumpLS"),
                    "RBumpLS", () -> new PathPlannerAuto("RBumpLS")),
            new AutoFamily("Bump LW",
                    "LBumpLW", () -> new PathPlannerAuto("LBumpLW"),
                    "RBumpLW", () -> new PathPlannerAuto("RBumpLW")),
            new AutoFamily("Bump MS",
                    "LBumpMS", () -> new PathPlannerAuto("LBumpMS"),
                    "RBumpMS", () -> new PathPlannerAuto("RBumpMS")),
            new AutoFamily("Bump MW",
                    "LBumpMW", () -> new PathPlannerAuto("LBumpMW"),
                    "RBumpMW", () -> new PathPlannerAuto("RBumpMW")),
            new AutoFamily("Bump SS",
                    "LBumpSS", () -> new PathPlannerAuto("LBumpSS"),
                    "RBumpSS (Auto-Mirrored)", () -> new PathPlannerAuto("LBumpSS", true)),
            new AutoFamily("Bump SW",
                    "LBumpSW", () -> new PathPlannerAuto("LBumpSW"),
                    "RBumpSW (Auto-Mirrored)", () -> new PathPlannerAuto("LBumpSW", true)));

    /** Every leftName/rightName across AUTO_DETECT_FAMILIES, so the raw per-side .auto options can be filtered out of the chooser in favor of the combined family entry. */
    public static Set<String> coveredAutoNames() {
        Set<String> names = new HashSet<>();
        for (AutoFamily family : AUTO_DETECT_FAMILIES) {
            names.add(family.leftName());
            names.add(family.rightName());
        }
        return names;
    }
}
