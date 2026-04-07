package frc.robot;

import java.util.Optional;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.LightsConstants;
import frc.robot.commands.Lights.WPIlib.ResetLED;
import frc.robot.commands.Lights.WPIlib.SetBlinkingPattern;
import frc.robot.subsystems.Lights.LEDSubsystem_WPIlib;

public class MatchInformation extends SubsystemBase {
    // private final LEDSubsystem_WPIlib normalLights;
    private final Timer matchTimer;

    // Match Phases
    public enum MatchPhase {
        DISABLED,
        AUTONOMOUS,
        TELEOP,
        ENDGAME,
        TRANSITION,
        UNKNOWN
    }

    // Driver Station Info - Required for ALL other calculations
    public boolean enabled;
    public boolean autonomous;
    public boolean teleop;
    public boolean disabled;
    public Optional<DriverStation.Alliance> alliance;
    public DriverStation.MatchType matchType;
    public int matchNumber;
    public double matchTime;
    public String gameData;

    // Phases + Shifts
    public MatchPhase phase;
    public double phaseElapsed;
    public boolean hubActive;
    public int teleopShift;
    public double shiftTimeRemaining;
    public static final double SHIFT_WARNING_THRESHOLD = 5;
    public boolean shift1Active;
    public boolean redInactiveFirst;
    public boolean shiftWarning_advised;
    public boolean shiftWarning_active;
    public double internalTeleopTime; // Internal teleop timer used for LED / shift logic
    // private boolean transitionSignalActive;

    private enum WarningStage {
        NONE,
        SLOW,
        FAST,
        SHIFT,
        ENDGAME
    }

    private WarningStage currentWarningStage;

    // Flags
    public boolean endgame;
    public boolean climbAdvised;
    public boolean scoringSafeWindow;

    // Time
    public double fpgaTimestamp;
    public double teleopStartTimestamp;
    public double autoStartTimestamp;

    public MatchInformation(/*LEDSubsystem_WPIlib m_normalLights,*/ Timer m_matchTime) {
        matchTimer = m_matchTime;
        // normalLights = m_normalLights;
        revertDefaultState();
    }

    /** Revert all match info to default state. */
    public void revertDefaultState() {
        enabled = false;
        autonomous = false;
        teleop = false;
        disabled = true;

        alliance = Optional.empty();
        matchType = DriverStation.MatchType.None;
        matchNumber = -1;
        matchTime = -1;
        gameData = "";

        phase = MatchPhase.DISABLED;
        shiftTimeRemaining = 0;
        phaseElapsed = 0;
        shiftWarning_active = false;

        hubActive = false;
        teleopShift = 0;
        shift1Active = false;
        redInactiveFirst = false;

        endgame = false;
        climbAdvised = false;
        scoringSafeWindow = false;
        currentWarningStage = WarningStage.NONE;
        // transitionSignalActive = false;

        fpgaTimestamp = 0;
        teleopStartTimestamp = -1;
        autoStartTimestamp = -1;
    }

    /**
     * Update [Driver Station] information. ! Run first - other data is based on
     * this. !
     */
    public void updateDriverStation() {
        enabled = DriverStation.isEnabled();
        autonomous = DriverStation.isAutonomous();
        teleop = DriverStation.isTeleop();
        disabled = DriverStation.isDisabled();

        alliance = DriverStation.getAlliance();
        matchType = DriverStation.getMatchType();
        matchNumber = DriverStation.getMatchNumber();
        matchTime = 140 - matchTimer.get();
        gameData = DriverStation.getGameSpecificMessage();
    }

    /**
     * Update timestamps and calculate elapsed time for phases. ! Run after updating
     * Driver Station info. !
     */
    public void updateTimestamps() {
        fpgaTimestamp = Timer.getFPGATimestamp();

        if (autonomous && autoStartTimestamp < 0)
            autoStartTimestamp = fpgaTimestamp;

        if (teleop && teleopStartTimestamp < 0)
            teleopStartTimestamp = fpgaTimestamp;

        if (disabled) {
            teleopStartTimestamp = -1;
            autoStartTimestamp = -1;
        }

        // Internal LED timer (countdown like FMS but based on FPGA time)
        if (teleopStartTimestamp >= 0) {
            double teleopElapsed = fpgaTimestamp - teleopStartTimestamp;
            internalTeleopTime = Math.max(0, 140.0 - teleopElapsed);
        } else {
            internalTeleopTime = -1;
        }
    }

    /**
     * Update the current match phase based on Driver Station info and timestamps. !
     * Run after updating Driver Station info and timestamps. !
     */
    public void updatePhase() {

        if (disabled) {
            phase = MatchPhase.DISABLED;
            phaseElapsed = 0;
            return;
        }

        if (autonomous) {
            phase = MatchPhase.AUTONOMOUS;
            if (autoStartTimestamp >= 0)
                phaseElapsed = fpgaTimestamp - autoStartTimestamp;
            return;
        }

        if (teleop) {

            if (matchTime > 130) {
                phase = MatchPhase.TRANSITION;
                endgame = false;
                phaseElapsed = 0;
                return;
            }

            if (matchTime >= 0 && matchTime <= 30) {
                phase = MatchPhase.ENDGAME;
                endgame = true;
            } else {
                phase = MatchPhase.TELEOP;
                endgame = false;
            }

            if (teleopStartTimestamp >= 0)
                phaseElapsed = fpgaTimestamp - teleopStartTimestamp;

            return;
        }

        phase = MatchPhase.UNKNOWN;
    }

    /**
     * Update hub and shift logic based on match time and game data. ! Run after
     * updating Driver Station info and timestamps. !
     */
    public void updateHubLogic(double teleopTime) {

        teleopShift = 0;
        shiftTimeRemaining = 0;

        // No alliance → cannot compute
        if (alliance.isEmpty()) {
            hubActive = false;
            return;
        }

        // Autonomous → hub always active
        if (autonomous) {
            hubActive = true;
            return;
        }

        // Not teleop → no hub logic
        if (!teleop || teleopTime < 0) {
            hubActive = true;
            return;
        }

        // Missing game data → assume active
        if (gameData.isEmpty()) {
            hubActive = true;
            return;
        }

        // Decode auto winner
        char c = gameData.charAt(0);
        if (c != 'R' && c != 'B') {
            hubActive = true;
            return;
        }

        redInactiveFirst = (c == 'R');

        shift1Active = switch (alliance.get()) {
            case Red -> !redInactiveFirst;
            case Blue -> redInactiveFirst;
        };

        // Shifts
        if (teleopTime > 130) {
            hubActive = true;
            return;
        } else if (teleopTime > 105) {
            teleopShift = 1;
            hubActive = shift1Active;
        } else if (teleopTime > 80) {
            teleopShift = 2;
            hubActive = !shift1Active;
        } else if (teleopTime > 55) {
            teleopShift = 3;
            hubActive = shift1Active;
        } else if (teleopTime > 30) {
            teleopShift = 4;
            hubActive = !shift1Active;
        } else {
            hubActive = true;
            return;
        }

        // Shift timing
        double cycleElapsed = 130 - teleopTime;
        double intoShift = cycleElapsed % 25.0;
        shiftTimeRemaining = 25.0 - intoShift;

        if (shiftTimeRemaining >= 25.0)
            shiftTimeRemaining = 0;
    }

    /**
     * Update convenience flags based on current match state. ! Run after updating
     * Driver Station info, timestamps, and hub logic. !
     * - climbAllowed: true during endgame, false otherwise
     * - scoringSafeWindow: true when hub is active and it's not endgame, false
     * otherwise
     */
    public void updateFlags() {
        climbAdvised = endgame;
        scoringSafeWindow = hubActive && !endgame;
    }

    public boolean isMatchConnected() {
        return enabled && matchTime >= 0;
    }

    /**
     * Update shift warning lights based on remaining shift time.
     */
    public void updateShiftWarning() {

        // ================================
        // ENDGAME
        // ================================
        if (endgame) {
            if (currentWarningStage != WarningStage.ENDGAME) {
                currentWarningStage = WarningStage.ENDGAME;
                // new SetBlinkingPattern(
                //         normalLights,
                //         LEDSubsystem_WPIlib.LEDTarget.SIDES,
                //         LEDPattern.solid(LightsConstants.RBGColors.get("yellow")),
                //         0.25,
                //         0.25).schedule();
            }
            return;
        }

        // ================================
        // INVALID CONDITIONS
        // ================================
        if (!teleop || teleopShift <= 0) {
            if (currentWarningStage != WarningStage.NONE) {
                currentWarningStage = WarningStage.NONE;

            //     new ResetLED(
            //             normalLights,
            //             LEDSubsystem_WPIlib.LEDTarget.SIDES).schedule();
            // }
            return;
        }
    }

        double t = shiftTimeRemaining;
        Color nextColor = hubActive
                ? LightsConstants.RBGColors.get("red")
                : LightsConstants.RBGColors.get("green");

        // ================================
        // MAGENTA SHIFT SIGNAL (25–23s)
        // ================================
        if (t >= 23 && t <= 25) {
            if (currentWarningStage != WarningStage.SHIFT) {
                currentWarningStage = WarningStage.SHIFT;
                // new SetBlinkingPattern(
                //         normalLights,
                //         LEDSubsystem_WPIlib.LEDTarget.SIDES,
                //         LEDPattern.solid(LightsConstants.RBGColors.get("magenta")),
                //         0.25,
                //         0.25).schedule();
            }
            return;
        }

        // ================================
        // SLOW WARNING (10–5s)
        // ================================
        if (t <= 10 && t > 5) {
            if (currentWarningStage != WarningStage.SLOW) {
                currentWarningStage = WarningStage.SLOW;
                // new SetBlinkingPattern(
                //         normalLights,
                //         LEDSubsystem_WPIlib.LEDTarget.SIDES,
                //         LEDPattern.solid(nextColor),
                //         0.5,
                //         0.5).schedule();
            }
            return;
        }

        // ================================
        // FAST WARNING (5–0s)
        // ================================
        if (t <= 5 && t > 0) {
            if (currentWarningStage != WarningStage.FAST) {
                currentWarningStage = WarningStage.FAST;
                // new SetBlinkingPattern(
                //         normalLights,
                //         LEDSubsystem_WPIlib.LEDTarget.SIDES,
                //         LEDPattern.solid(nextColor),
                //         0.25,
                //         0.25).schedule();
            }
            return;
        }

        // ================================
        // OUTSIDE WARNING WINDOW (23-10s)
        // ================================
        if (currentWarningStage != WarningStage.NONE) {
            currentWarningStage = WarningStage.NONE;
            // new ResetLED(
            //         normalLights,
            //         LEDSubsystem_WPIlib.LEDTarget.SIDES).schedule();
        } else {
            return;
        }
    }

    /** Periodic update wrapper. */
    public void updateAll() {
        updateDriverStation();
        updateTimestamps();
        updatePhase();
        updateHubLogic(internalTeleopTime);
        updateFlags();
        updateShiftWarning();
    }

    @Override
    public void periodic() {
        updateAll();
    }
}