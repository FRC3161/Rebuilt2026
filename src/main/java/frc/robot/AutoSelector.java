package frc.robot;

import java.io.IOException;
import java.util.ArrayList;
import java.util.HashSet;
import java.util.List;
import java.util.Set;
import java.util.function.Supplier;

import org.json.simple.parser.ParseException;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.PathPlannerPath;
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
 * coveredAutoNames() below) — adding an entry to AUTO_DETECT_FAMILIES is the only step needed
 * to combine two already-validated hand-authored autos, or to generate one side via mirror
 * from the other; do not add a pair there without verifying the underlying .traj Y-values
 * actually mirror around FlippingUtil.fieldSizeY / 2.
 *
 * For a brand new auto that just needs a generated mirror and nothing else, no registration
 * is needed at all: name the file "Left:<Name>" and discoverPrefixedFamilies() below will
 * find it, generate the mirror, and combine it into the chooser as "<Name>" automatically.
 * The colon is deliberate -- no existing hand-authored auto uses one, so this convention can
 * never collide with the manually-curated list above. This still generates the mirror
 * unconditionally with no human review of whether the path is actually safe to mirror, so it
 * is only appropriate for genuinely one-off, low-stakes autos -- anything that needs a real
 * "is this a valid mirror" judgment call belongs in AUTO_DETECT_FAMILIES instead.
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

    private static final String MIRROR_PREFIX = "Left:";

    /**
     * Scans every deployed .auto file for the "Left:<Name>" naming convention and builds a
     * generated-mirror AutoFamily for each one found, with no manual registration required.
     * Unlike AUTO_DETECT_FAMILIES, the Right side is always generated via mirror=true here --
     * there is no way to point this at an existing hand-authored file, since the whole point
     * is skipping registration entirely for simple cases.
     */
    public static List<AutoFamily> discoverPrefixedFamilies() {
        List<AutoFamily> discovered = new ArrayList<>();
        for (String autoName : AutoBuilder.getAllAutoNames()) {
            if (!autoName.startsWith(MIRROR_PREFIX)) {
                continue;
            }
            String displayName = autoName.substring(MIRROR_PREFIX.length());
            discovered.add(new AutoFamily(
                    displayName,
                    autoName, () -> new PathPlannerAuto(autoName),
                    displayName + " (Auto-Mirrored)", () -> new PathPlannerAuto(autoName, true)));
        }
        return discovered;
    }

    /** Every leftName/rightName across the given families, so the raw per-side .auto options can be filtered out of the chooser in favor of the combined family entry. */
    public static Set<String> coveredAutoNames(List<AutoFamily> families) {
        Set<String> names = new HashSet<>();
        for (AutoFamily family : families) {
            names.add(family.leftName());
            names.add(family.rightName());
        }
        return names;
    }

    /**
     * Poses for every point of every path in the named auto, for a pre-match preview on a
     * Field2d widget -- re-parses the .auto file directly (PathPlannerAuto.getPathGroupFromAutoFile),
     * the same API PathPlannerLib documents for this exact use, so it works while just sitting
     * disabled and doesn't require actually scheduling/running the auto.
     *
     * Applies FlippingUtil.flipFieldPose on Red, the exact same transform AutoBuilder.resetOdom
     * applies at execution time (see CommandSwerveDrivetrain's shouldFlipPath config) -- without
     * this, the preview would only be correct on Blue, since getPathGroupFromAutoFile and
     * mirrorPath() both work in raw, unflipped field coordinates.
     */
    public static List<Pose2d> previewPoses(String autoName, boolean mirror, Alliance alliance)
            throws IOException, ParseException {
        List<Pose2d> poses = new ArrayList<>();
        for (PathPlannerPath path : PathPlannerAuto.getPathGroupFromAutoFile(autoName)) {
            for (Pose2d pose : (mirror ? path.mirrorPath() : path).getPathPoses()) {
                poses.add(alliance == Alliance.Red ? FlippingUtil.flipFieldPose(pose) : pose);
            }
        }
        return poses;
    }

    /**
     * Preview poses for whichever side of a family is currently in play. leftName is always a
     * real hand-authored file (true across every AUTO_DETECT_FAMILIES entry and everything
     * discoverPrefixedFamilies() finds). rightName is either a real hand-authored file too, or
     * a synthetic "(Auto-Mirrored)" label with no file on disk -- checked directly against
     * AutoBuilder.getAllAutoNames() rather than inferred from the label text, since a fragile
     * naming guess is exactly the kind of risk this class avoids elsewhere.
     */
    public static List<Pose2d> previewFamilySide(AutoFamily family, Side side, Alliance alliance)
            throws IOException, ParseException {
        if (side == Side.LEFT) {
            return previewPoses(family.leftName(), false, alliance);
        }
        boolean rightIsRealFile = AutoBuilder.getAllAutoNames().contains(family.rightName());
        return rightIsRealFile
                ? previewPoses(family.rightName(), false, alliance)
                : previewPoses(family.leftName(), true, alliance);
    }
}
