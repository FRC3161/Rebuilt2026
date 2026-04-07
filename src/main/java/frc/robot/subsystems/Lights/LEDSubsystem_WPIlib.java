package frc.robot.subsystems.Lights;

import static edu.wpi.first.units.Units.Percent;
import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Seconds;

import java.util.Random;

import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.AddressableLEDBufferView;
//LED Imports
//import edu.wpi.first.units.Units.*;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.LightsConstants;
import frc.robot.Constants.LightsConstants.Colors;

public class LEDSubsystem_WPIlib extends SubsystemBase {
    // edu.wpi.first.wpilibj.AddressableLED
    // A class for driving addressable LEDs, such as WS2812Bs and NeoPixels.
    // By default, the timing supports WS2812B LEDs, but is configurable using
    // setBitTiming()
    // Only 1 LED driver is currently supported by the roboRIO.
    // However, multiple LED strips can be connected in series and controlled from
    // the single driver.

    // SETUP
    // ========================================================================
    private static final int kPort = LightsConstants.led_port; // PWM Port
    private static final int kLength = LightsConstants.led_length; // LED strip length [# of LEDs]
    private static final Distance kLedSpacing = LightsConstants.spacing; // LED strip LEDs density - [... LEDs per
                                                                         // meter]
    public static int brightness = LightsConstants.led_brightness;
    private static final int signal_length = LightsConstants.signal_length; // Length of signal LED sector (last x LEDs)

    // BUFFERS
    // ======================================================================
    private final AddressableLED m_led;
    private final AddressableLEDBuffer m_ledbuffer; // The entire LED strip
    private final AddressableLEDBufferView m_sidesBuffer; // The buffer for sides of the robot
    private final AddressableLEDBufferView m_signalBuffer; // The buffer for the signal LEDs (last [signal_length] LEDs)

    // HANDLERS =============================================================
    private final Random random = new Random();

    public enum LEDTarget {
        SIDES,
        SIGNAL
    }

    // LED BUFFER STATES ====================================================
    private class LEDState {
        Timer timer = new Timer();
        boolean running_AnimatedPattern = false;
        LEDPattern animatedPattern;
        // Twinkle pattern state variables
        boolean running_TwinklePattern = false;
        double twinklePeriod = 0;
        Color twinkleBaseColor = null;
        Color twinkleColor = null;
        boolean[] twinkleMask; // fast lookup instead of List
        double[] twinklePhaseOffset; // each LED has independent phase
        int twinkle_Count = 10; // number of LEDs to twinkle at a time, can be tuned
    }

    private final LEDState m_signalBuffer_State = new LEDState();
    private final LEDState m_sidesBuffer_State = new LEDState();

    public LEDSubsystem_WPIlib() {
        // LED AND BUFFERS ============================================================
        m_led = new AddressableLED(kPort);
        m_ledbuffer = new AddressableLEDBuffer(kLength);
        m_sidesBuffer = m_ledbuffer.createView(0, kLength - signal_length - 1); // Sides of the robot
        m_signalBuffer = m_ledbuffer.createView(kLength - signal_length, kLength - 1); // Signal LEDs (last [signal_length] LEDs)

        m_led.setLength(m_ledbuffer.getLength());
        m_led.setData(m_ledbuffer);
        m_led.start();

        // To get color, use: LightsConstants.RBGColors.get("gold")

        // LED_Breathing(LEDTarget.SIDES,
        // LEDPattern.solid(LightsConstants.RBGColors.get("red")), 2.5);
        LED_Twinkle(LEDTarget.SIDES, LightsConstants.RBGColors.get("black"), LightsConstants.RBGColors.get("gold"),
                2.5);
        // LED_ScrollPatternRelative(LEDPattern.rainbow(255, 120), 100);
    }

    /**
     * Disable LED strip - Terminates all patterns and stops the LED strip.
     * !! DO NOT USE UNLESS NECESSARY !! - It fully disables LEDs until robot
     * restart.
     */
    public void LED_Disable() {
        stopTwinkle(LEDTarget.SIDES);
        stopTwinkle(LEDTarget.SIGNAL);
        runPattern(LEDTarget.SIDES, LEDPattern.solid(LightsConstants.GRBColors.get("black")), false);
        runPattern(LEDTarget.SIGNAL, LEDPattern.solid(LightsConstants.GRBColors.get("black")), false);
        m_led.stop();
    }

    /**
     * Resetting LED strip - LED set to solid black.
     */
    public void LED_Reset(LEDTarget target) {
        stopTwinkle(target);
        runPattern(target, LEDPattern.solid(LightsConstants.GRBColors.get("black")), false);
    }

    /**
     * Solid color pattern.
     * 
     * @param color the color of the pattern
     */
    public void LED_SolidColor(LEDTarget target, Color color) {
        runPattern(target, LEDPattern.solid(color), false);
    }

    /**
     * Scrolling pattern at relative speed.
     * 
     * @param pattern   the LED pattern to run
     * @param magnitude for frequency calculation [/s]
     */
    public void LED_ScrollPatternRelative(LEDTarget target, LEDPattern pattern, double magnitude) {
        // LEDPattern m_scrollingPattern =
        // pattern.scrollAtAbsoluteSpeed(MetersPerSecond.of(speed), kLedSpacing);
        LEDPattern m_scrollingPattern = pattern.scrollAtRelativeSpeed(Percent.per(Second).of(magnitude));
        runPattern(target, m_scrollingPattern, true);
        // System.out.println("Scroll function executed!!!!!!!!!!!!");
    }

    /**
     * Blinking pattern.
     * 
     * @param pattern the LED pattern to run
     * @param onTime  the time the LED stays on in [s]
     * @param offTime the time the LED stays off in [s]
     */
    public void LED_Blinking(LEDTarget target, LEDPattern pattern, double onTime, double offTime) {
        LEDPattern m_blinkingPattern = pattern.blink(Seconds.of(onTime), Seconds.of(offTime));
        runPattern(target, m_blinkingPattern, true);
    }

    /**
     * Breathing pattern.
     * 
     * @param pattern the LED pattern to run
     * @param period  the time of one full cycle in [s]
     */
    public void LED_Breathing(LEDTarget target, LEDPattern pattern, double period) {
        LEDPattern breathing = pattern.breathe(Seconds.of(period));
        runPattern(target, breathing, true);
    }

    /**
     * Twinkle pattern.
     * 
     * @param baseColor    the color of static LED's
     * @param twinkleColor the color of twinkling LED's
     * @param period       the time of one full cycle in [s]
     */
    public void LED_Twinkle(
            LEDTarget target,
            Color baseColor,
            Color twinkleColor,
            double period) {

        LEDState state = getState(target);
        AddressableLEDBufferView buffer = getBuffer(target);

        state.running_TwinklePattern = true;
        state.running_AnimatedPattern = false;

        state.timer.reset();
        state.timer.start();

        state.twinkleBaseColor = baseColor;
        state.twinkleColor = twinkleColor;
        state.twinklePeriod = period;

        int length = buffer.getLength();
        state.twinkleMask = new boolean[length];
        state.twinklePhaseOffset = new double[length];

        randomizeTwinkleLEDs(state, length);
    }

    /**
     * Stops the twinkle effect by resetting all twinkle-related state.
     */
    public void stopTwinkle(LEDTarget target) {

        LEDState state = getState(target);

        state.running_TwinklePattern = false;

        if (state.timer.isRunning()) {
            state.timer.stop();
            state.timer.reset();
        }

        if (state.twinkleMask != null) {
            for (int i = 0; i < state.twinkleMask.length; i++) {
                state.twinkleMask[i] = false;
                state.twinklePhaseOffset[i] = 0;
            }
        }
    }

    @Override
    public void periodic() {
        updateBuffer(m_sidesBuffer, m_sidesBuffer_State);
        updateBuffer(m_signalBuffer, m_signalBuffer_State);
        m_led.setData(m_ledbuffer);
    }

    /**
     * Updates the LED buffer based on the current state of the patterns.
     * It applies the animated pattern if it's running, and then applies the
     * twinkle effect if it's active.
     * 
     * @param buffer the LED buffer to update
     * @param state  the current state of the LED patterns
     */
    private void updateBuffer(AddressableLEDBufferView buffer,
            LEDState state) {

        if (state.running_AnimatedPattern && state.animatedPattern != null) {
            state.animatedPattern.applyTo(buffer);
        }

        if (state.running_TwinklePattern) {

            double t = state.timer.get();

            for (int i = 0; i < buffer.getLength(); i++) {

                if (state.twinkleMask[i]) {

                    double phase = ((t + state.twinklePhaseOffset[i])
                            % state.twinklePeriod) / state.twinklePeriod;

                    double b = 0.5 - 0.5 * Math.cos(phase * 2 * Math.PI);

                    Color c = new Color(
                            state.twinkleColor.red * b,
                            state.twinkleColor.green * b,
                            state.twinkleColor.blue * b);

                    buffer.setLED(i, c);

                } else if (!state.running_AnimatedPattern) {

                    buffer.setLED(i, state.twinkleBaseColor);
                }
            }

            if (t >= state.twinklePeriod) {
                randomizeTwinkleLEDs(state, buffer.getLength());
                state.timer.reset();
            }
        }
    }

    /**
     * Random assignment of LEDs
     */
    private void randomizeTwinkleLEDs(LEDState state, int length) {

        for (int i = 0; i < length; i++) {
            state.twinkleMask[i] = false;
        }

        int max = Math.min(state.twinkle_Count, length);

        for (int i = 0; i < max; i++) {
            int index = random.nextInt(length);
            state.twinkleMask[index] = true;
            state.twinklePhaseOffset[index] = -random.nextDouble() * state.twinklePeriod * 0.5;
        }
    }

    /**
     * A function that runs a pattern on the entire LED strip.
     * It also controls whether the pattern is animated or not
     * using the running_AnimatedPattern flag and animatedPattern.
     * 
     * 
     * @param pattern  the LED pattern to run
     * @param animated whether the pattern is animated
     * @implNote Recomended pattern input (rainbow): private final LEDPattern
     *           m_rainbow = LEDPattern.rainbow(255, 128);
     */
    public void runPattern(LEDTarget target, LEDPattern pattern, boolean animated) {
        LEDState state = getState(target);
        AddressableLEDBufferView buffer = getBuffer(target);

        // Stop twinkle only for this buffer
        state.running_TwinklePattern = false;
        state.timer.stop();
        state.timer.reset();

        if (animated) {
            state.animatedPattern = pattern.atBrightness(Percent.of(brightness));
            state.running_AnimatedPattern = true;
        } else {
            state.animatedPattern = null;
            state.running_TwinklePattern = false;
            pattern.atBrightness(Percent.of(brightness)).applyTo(buffer);
        }
    }

    /**
     * Helper functions to get the state and buffer based on the target enum.
     */
    private LEDState getState(LEDTarget target) {
        return target == LEDTarget.SIDES
                ? m_sidesBuffer_State
                : m_signalBuffer_State;
    }

    /**
     * Helper function to get the buffer based on the target enum.
     */
    private AddressableLEDBufferView getBuffer(LEDTarget target) {
        return target == LEDTarget.SIDES
                ? m_sidesBuffer
                : m_signalBuffer;
    }

    public void updateBrightness(int newBrightness) {
        LEDSubsystem_WPIlib.brightness = newBrightness;
    }
}

/**
 * 
 * AddressableLED m_led = new AddressableLED(0);
 * //----------------------------------------------------------------------------------------------------------------------
 * AddressableLEDBuffer m_ledBuffer = new AddressableLEDBuffer(60);
 * m_led.setLength(m_ledBuffer.getLength());
 * 
 * // Set the data
 * m_led.setData(m_ledBuffer);
 * m_led.start();
 * 
 * // Create the view for the section of the strip on the left side of the
 * robot.
 * // This section spans LEDs from index 0 through index 59, inclusive.
 * AddressableLEDBufferView m_left = m_ledBuffer.createView(0, 29);
 * 
 * // The section of the strip on the right side of the robot.
 * // This section spans LEDs from index 60 through index 119, inclusive.
 * // This view is reversed to cancel out the serpentine arrangement of the
 * // physical LED strip on the robot.
 * AddressableLEDBufferView m_right = m_ledBuffer.createView(30, 59).reversed();
 * 
 * //Patterns
 * 
 * //LED pattern that sets the entire strip to solid red
 * LEDPattern red = LEDPattern.solid(Color.kRed);
 * red.applyTo(m_ledBuffer);
 * m_led.setData(m_ledBuffer);
 * 
 * //Animated Rainbow
 * // all hues at maximum saturation and half brightness
 * private final LEDPattern m_rainbow = LEDPattern.rainbow(255, 128);
 * // Our LED strip has a density of 120 LEDs per meter
 * private static final Distance kLedSpacing = Meters.of(1 / 120.0);
 * // Pattern that scrolls the rainbow pattern across the LED strip, moving at a
 * speed
 * // of 1 meter per second.
 * private final LEDPattern m_scrollingRainbow =
 * m_rainbow.scrollAtAbsoluteSpeed(MetersPerSecond.of(1), kLedSpacing);
 * //In periodic
 * m_scrollingRainbow.applyTo(m_ledBuffer);
 * m_led.setData(m_ledBuffer);
 * 
 * //
 */