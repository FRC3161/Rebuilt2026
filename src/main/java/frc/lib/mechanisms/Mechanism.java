package frc.lib.mechanisms;

import java.util.ArrayList;
import java.util.List;
import java.util.Map;
import java.util.concurrent.ConcurrentHashMap;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.sim.ChassisReference;
import com.ctre.phoenix6.sim.TalonFXSimState;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.lib.hardware.CanIdRegistry;
import frc.lib.sim.SimBattery;
import frc.lib.util.LoggedTunableNumber;
import frc.lib.util.PhoenixUtil;

/**
 * A motor-driven mechanism: one leader TalonFX plus any followers, already
 * configured, simulated, tunable and logged. Not a subsystem on its own --
 * subsystems own one or more mechanisms and register them so they get
 * updated every loop (see StateMachineSubsystem).
 *
 * Subclasses (Roller, Flywheel, Pivot, Linear, Turret) only add the
 * physics model for simulation, a dashboard drawing, and control methods
 * with names that make sense for that kind of mechanism.
 */
public abstract class Mechanism implements AutoCloseable {
    public enum ControlMode {
        NEUTRAL, DUTY_CYCLE, VOLTAGE, POSITION, VELOCITY
    }

    /** One step of simulated physics, in mechanism units. */
    protected record SimOutput(double position, double velocity, double currentAmps) {
    }

    protected final MechanismConfig config;
    protected final String name;
    protected final TalonFX leader;
    private final List<TalonFX> followers = new ArrayList<>();
    private final TalonFXConfiguration talonConfig = new TalonFXConfiguration();

    // Signals, refreshed together once per loop
    private final StatusSignal<Angle> positionSignal;
    private final StatusSignal<AngularVelocity> velocitySignal;
    private final StatusSignal<Voltage> voltageSignal;
    private final StatusSignal<Current> statorSignal;
    private final StatusSignal<Current> supplySignal;
    private boolean connected = true;

    // Control requests, reused every loop
    private final DutyCycleOut dutyCycleRequest = new DutyCycleOut(0);
    private final VoltageOut voltageRequest = new VoltageOut(0);
    private final PositionVoltage positionRequest = new PositionVoltage(0);
    private final MotionMagicVoltage motionMagicRequest = new MotionMagicVoltage(0);
    private final VelocityVoltage velocityRequest = new VelocityVoltage(0);
    private final NeutralOut neutralRequest = new NeutralOut();
    private ControlMode controlMode = ControlMode.NEUTRAL;
    private double setpoint = 0;

    // Live tuning (only for closed-loop mechanisms)
    private final List<LoggedTunableNumber> tunables = new ArrayList<>();
    private LoggedTunableNumber tunableP, tunableI, tunableD, tunableS, tunableV, tunableA, tunableG;
    private LoggedTunableNumber tunableCruise, tunableAccel;

    // Simulation
    private final List<TalonFXSimState> simStates = new ArrayList<>();
    private double lastSimPosition;

    // Dashboard drawing
    private final Mechanism2d visual = new Mechanism2d(3, 3);

    // Every live mechanism by name, so robot-wide code (like the 3D robot
    // view's RobotComponents) can read positions without reaching into subsystems.
    private static final Map<String, Mechanism> BY_NAME = new ConcurrentHashMap<>();

    /** The live mechanism with this name ("Subsystem/Part"), or null if none was created. */
    public static Mechanism find(String name) {
        return BY_NAME.get(name);
    }

    protected Mechanism(MechanismConfig config) {
        config.require(config.leaderId >= 0, "needs withLeader(canId)");
        this.config = config;
        this.name = config.name;

        // Claim the ID before creating the device, so a duplicate fails without touching the bus.
        CanIdRegistry.claim(config.canBus.getName(), config.leaderId, name);
        leader = new TalonFX(config.leaderId, config.canBus);
        BY_NAME.put(name, this);
        for (MechanismConfig.FollowerSpec spec : config.followers) {
            CanIdRegistry.claim(config.canBus.getName(), spec.id(), name + " (follower)");
            followers.add(new TalonFX(spec.id(), config.canBus));
        }

        positionSignal = leader.getPosition();
        velocitySignal = leader.getVelocity();
        voltageSignal = leader.getMotorVoltage();
        statorSignal = leader.getStatorCurrent();
        supplySignal = leader.getSupplyCurrent();

        lastSimPosition = config.startingPosition;
        positionRequest.Slot = 0;
        motionMagicRequest.Slot = 0;
        velocityRequest.Slot = 0;
    }

    /**
     * Applies the config to the motors. Called by each subclass at the end of
     * its constructor, once its sim model exists -- not from this constructor,
     * because subclass fields aren't set yet at that point.
     */
    protected final void initialize() {
        buildTalonConfig();
        PhoenixUtil.tryUntilOk("configure " + name, () -> leader.getConfigurator().apply(talonConfig));
        for (int i = 0; i < followers.size(); i++) {
            TalonFX follower = followers.get(i);
            MotorAlignmentValue alignment = config.followers.get(i).alignment();
            PhoenixUtil.tryUntilOk("configure " + name + " follower",
                    () -> follower.getConfigurator().apply(talonConfig));
            follower.setControl(new Follower(config.leaderId, alignment));
        }
        PhoenixUtil.tryUntilOk("seed " + name + " encoder", () -> leader.setPosition(config.startingPosition));

        if (isTunable()) {
            createTunables();
        }
        if (RobotBase.isSimulation()) {
            createSimStates();
        }
        visual.getRoot(name, 1.5, 1.5).append(createVisual());
        SmartDashboard.putData(name + "/Visual", visual);
    }

    private void buildTalonConfig() {
        talonConfig.MotorOutput.Inverted = config.inverted
                ? InvertedValue.Clockwise_Positive
                : InvertedValue.CounterClockwise_Positive;
        talonConfig.MotorOutput.NeutralMode = config.brakeMode ? NeutralModeValue.Brake : NeutralModeValue.Coast;

        talonConfig.CurrentLimits.SupplyCurrentLimit = config.supplyCurrentLimit;
        talonConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
        talonConfig.CurrentLimits.StatorCurrentLimit = config.statorCurrentLimit;
        talonConfig.CurrentLimits.StatorCurrentLimitEnable = true;

        talonConfig.Feedback.SensorToMechanismRatio = sensorToMechanismRatio();

        talonConfig.Slot0 = new Slot0Configs()
                .withKP(config.kP).withKI(config.kI).withKD(config.kD)
                .withKS(config.kS).withKV(config.kV).withKA(config.kA).withKG(gravityInSlot() ? config.kG : 0);
        talonConfig.MotionMagic = new MotionMagicConfigs()
                .withMotionMagicCruiseVelocity(config.motionMagicCruiseVelocity)
                .withMotionMagicAcceleration(config.motionMagicAcceleration)
                .withMotionMagicJerk(config.motionMagicJerk);

        if (useSoftLimits() && config.hasLimits()) {
            talonConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
            talonConfig.SoftwareLimitSwitch.ForwardSoftLimitThreshold = config.maxPosition;
            talonConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
            talonConfig.SoftwareLimitSwitch.ReverseSoftLimitThreshold = config.minPosition;
        }

        customizeConfig(talonConfig);
        config.extraConfig.accept(talonConfig);
    }

    /* ---------------- Hooks for subclasses ---------------- */

    /** Motor rotations per one mechanism unit. Linear mechanisms override this to work in meters. */
    protected double sensorToMechanismRatio() {
        return config.gearRatio;
    }

    /** Whether the min/max limits become motor soft limits. Rollers with simulated hard stops say no. */
    protected boolean useSoftLimits() {
        return true;
    }

    /** Whether to publish live-tunable gains. */
    protected boolean isTunable() {
        return true;
    }

    /** Last chance to adjust the TalonFX config (e.g. gravity type). */
    protected void customizeConfig(TalonFXConfiguration talonConfig) {
    }

    /**
     * Whether kG goes to the motor controller's slot gains. Pivots say no and
     * add gravity themselves through {@link #positionFeedforward()}, so their
     * 0 and positive direction can be anything.
     */
    protected boolean gravityInSlot() {
        return true;
    }

    /** Extra volts added to every position request (pivots: gravity). Recomputed each loop. */
    protected double positionFeedforward() {
        return 0;
    }

    /** The current kG: the live-tuned value if tuning is on, else the configured one. */
    protected double gravityGain() {
        return tunableG != null ? tunableG.get() : config.kG;
    }

    /** Advances the physics model by dt seconds with the given motor voltage. */
    protected abstract SimOutput simulate(double motorVolts, double dtSeconds);

    /** Creates the ligament drawn on the dashboard. */
    protected abstract MechanismLigament2d createVisual();

    /** Updates the dashboard drawing to the current state. */
    protected abstract void updateVisual();

    /** The combined motor model for every motor in this mechanism, for physics sims. */
    protected DCMotor gearbox() {
        return config.motor.model(config.motorCount());
    }

    /* ---------------- Open-loop control ---------------- */

    public void setDutyCycle(double dutyCycle) {
        controlMode = ControlMode.DUTY_CYCLE;
        setpoint = dutyCycle;
        leader.setControl(dutyCycleRequest.withOutput(dutyCycle));
    }

    public void setVoltage(double volts) {
        controlMode = ControlMode.VOLTAGE;
        setpoint = volts;
        leader.setControl(voltageRequest.withOutput(volts));
    }

    /** Stops driving the motor (it brakes or coasts per the config). */
    public void stop() {
        controlMode = ControlMode.NEUTRAL;
        setpoint = 0;
        leader.setControl(neutralRequest);
    }

    /* ---------------- Closed-loop control (exposed by subclasses) ---------------- */

    /** Drives to a position, clamped to the limits. Uses Motion Magic if a profile is configured. */
    protected void runPosition(double position) {
        if (config.hasLimits()) {
            position = MathUtil.clamp(position, config.minPosition, config.maxPosition);
        }
        controlMode = ControlMode.POSITION;
        setpoint = position;
        double feedforward = positionFeedforward();
        if (config.usesMotionMagic()) {
            leader.setControl(motionMagicRequest.withPosition(position).withFeedForward(feedforward));
        } else {
            leader.setControl(positionRequest.withPosition(position).withFeedForward(feedforward));
        }
    }

    protected void runVelocity(double velocity) {
        controlMode = ControlMode.VELOCITY;
        setpoint = velocity;
        leader.setControl(velocityRequest.withVelocity(velocity));
    }

    /* ---------------- Readings ---------------- */

    public double getPosition() {
        return BaseStatusSignal.getLatencyCompensatedValueAsDouble(positionSignal, velocitySignal);
    }

    public double getVelocity() {
        return velocitySignal.getValueAsDouble();
    }

    public double getAppliedVolts() {
        return voltageSignal.getValueAsDouble();
    }

    public double getStatorCurrent() {
        return statorSignal.getValueAsDouble();
    }

    public double getSupplyCurrent() {
        return supplySignal.getValueAsDouble();
    }

    public boolean isConnected() {
        return connected;
    }

    public ControlMode getControlMode() {
        return controlMode;
    }

    /** The last commanded value, in the units of the current control mode. */
    public double getSetpoint() {
        return setpoint;
    }

    /** True when a position or velocity setpoint has been reached (within tolerance). */
    public boolean isAtSetpoint() {
        return switch (controlMode) {
            case POSITION -> Math.abs(getPosition() - setpoint) < config.tolerance;
            case VELOCITY -> Math.abs(getVelocity() - setpoint) < config.tolerance;
            default -> false;
        };
    }

    /** True when pushing hard but not moving (needs withStallDetection). */
    public boolean isStalled() {
        return config.stallCurrent > 0
                && Math.abs(getVelocity()) < config.stallVelocity
                && Math.abs(getStatorCurrent()) > config.stallCurrent;
    }

    /** Tells the encoder the mechanism is at this position (e.g. after homing). */
    public void resetPosition(double position) {
        leader.setPosition(position);
    }

    public String getName() {
        return name;
    }

    public MechanismConfig getConfig() {
        return config;
    }

    /* ---------------- Eco mode ---------------- */

    /** Swaps to the lower eco current limits (true) or back to the normal ones (false). */
    public void setEcoMode(boolean eco) {
        CurrentLimitsConfigs limits = talonConfig.CurrentLimits;
        limits.SupplyCurrentLimit = eco ? config.ecoSupplyCurrentLimit : config.supplyCurrentLimit;
        limits.StatorCurrentLimit = eco ? config.ecoStatorCurrentLimit : config.statorCurrentLimit;
        leader.getConfigurator().apply(limits);
        for (TalonFX follower : followers) {
            follower.getConfigurator().apply(limits);
        }
    }

    /* ---------------- Loop ---------------- */

    /** Reads sensors and applies any gains changed on the dashboard. Call at the start of each loop. */
    public void updateInputs() {
        connected = BaseStatusSignal.refreshAll(
                positionSignal, velocitySignal, voltageSignal, statorSignal, supplySignal).isOK();
        if (!tunables.isEmpty()) {
            applyChangedTunables();
        }
        // A feedforward that depends on position (pivot gravity) has to be
        // re-sent as the mechanism moves, even if nobody commands a new target.
        if (controlMode == ControlMode.POSITION && !gravityInSlot()) {
            runPosition(setpoint);
        }
    }

    /** Publishes this mechanism's values to the dashboard. Call at the end of each loop. */
    public void updateTelemetry() {
        SmartDashboard.putNumber(name + "/Position", getPosition());
        SmartDashboard.putNumber(name + "/Velocity", getVelocity());
        SmartDashboard.putString(name + "/Control Mode", controlMode.toString());
        SmartDashboard.putNumber(name + "/Setpoint", setpoint);
        SmartDashboard.putBoolean(name + "/At Setpoint", isAtSetpoint());
        SmartDashboard.putNumber(name + "/Applied Volts", getAppliedVolts());
        SmartDashboard.putNumber(name + "/Stator Current", getStatorCurrent());
        SmartDashboard.putNumber(name + "/Supply Current", getSupplyCurrent());
        SmartDashboard.putBoolean(name + "/Connected", connected);
        if (config.stallCurrent > 0) {
            SmartDashboard.putBoolean(name + "/Stalled", isStalled());
        }
        updateVisual();
    }

    /** Advances the simulated motors. Called automatically in sim by StateMachineSubsystem. */
    public void simulationPeriodic(double dtSeconds) {
        double battery = RobotController.getBatteryVoltage();
        for (TalonFXSimState sim : simStates) {
            sim.setSupplyVoltage(battery);
        }
        SimOutput out = simulate(simStates.get(0).getMotorVoltage(), dtSeconds);

        double ratio = sensorToMechanismRatio();
        double rotorDelta = (out.position() - lastSimPosition) * ratio;
        lastSimPosition = out.position();
        for (TalonFXSimState sim : simStates) {
            sim.addRotorPosition(rotorDelta);
            sim.setRotorVelocity(out.velocity() * ratio);
        }
        SimBattery.reportCurrent(name, out.currentAmps());
    }

    private void createSimStates() {
        ChassisReference leaderOrientation = config.inverted
                ? ChassisReference.Clockwise_Positive
                : ChassisReference.CounterClockwise_Positive;
        ChassisReference opposite = config.inverted
                ? ChassisReference.CounterClockwise_Positive
                : ChassisReference.Clockwise_Positive;

        TalonFXSimState leaderSim = leader.getSimState();
        leaderSim.Orientation = leaderOrientation;
        leaderSim.setMotorType(config.motor.simType());
        simStates.add(leaderSim);
        for (int i = 0; i < followers.size(); i++) {
            TalonFXSimState sim = followers.get(i).getSimState();
            sim.Orientation = config.followers.get(i).alignment() == MotorAlignmentValue.Opposed
                    ? opposite
                    : leaderOrientation;
            sim.setMotorType(config.motor.simType());
            simStates.add(sim);
        }
    }

    private void createTunables() {
        String prefix = name + "/";
        tunableP = tunable(prefix + "kP", config.kP);
        tunableI = tunable(prefix + "kI", config.kI);
        tunableD = tunable(prefix + "kD", config.kD);
        tunableS = tunable(prefix + "kS", config.kS);
        tunableV = tunable(prefix + "kV", config.kV);
        tunableA = tunable(prefix + "kA", config.kA);
        tunableG = tunable(prefix + "kG", config.kG);
        if (config.usesMotionMagic()) {
            tunableCruise = tunable(prefix + "Cruise Velocity", config.motionMagicCruiseVelocity);
            tunableAccel = tunable(prefix + "Acceleration", config.motionMagicAcceleration);
        }
    }

    private LoggedTunableNumber tunable(String key, double defaultValue) {
        LoggedTunableNumber number = new LoggedTunableNumber(key, defaultValue);
        tunables.add(number);
        return number;
    }

    private void applyChangedTunables() {
        // Check every tunable (hasChanged() has to be called on each one to
        // keep its "previous value" current), then apply once if any moved.
        boolean changed = false;
        for (LoggedTunableNumber number : tunables) {
            changed |= number.hasChanged();
        }
        if (!changed) {
            return;
        }
        talonConfig.Slot0
                .withKP(tunableP.get()).withKI(tunableI.get()).withKD(tunableD.get())
                .withKS(tunableS.get()).withKV(tunableV.get()).withKA(tunableA.get())
                .withKG(gravityInSlot() ? tunableG.get() : 0);
        leader.getConfigurator().apply(talonConfig.Slot0);
        if (tunableCruise != null) {
            talonConfig.MotionMagic
                    .withMotionMagicCruiseVelocity(tunableCruise.get())
                    .withMotionMagicAcceleration(tunableAccel.get());
            leader.getConfigurator().apply(talonConfig.MotionMagic);
        }
    }

    @Override
    public void close() {
        BY_NAME.remove(name, this);
        leader.close();
        CanIdRegistry.release(config.canBus.getName(), config.leaderId);
        for (int i = 0; i < followers.size(); i++) {
            followers.get(i).close();
            CanIdRegistry.release(config.canBus.getName(), config.followers.get(i).id());
        }
    }
}
