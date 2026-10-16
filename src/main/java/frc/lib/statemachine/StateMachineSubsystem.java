package frc.lib.statemachine;

import java.util.ArrayList;
import java.util.List;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.mechanisms.Mechanism;

/**
 * The team's subsystem pattern: callers ask for a WANTED state, and the
 * subsystem decides each loop which SYSTEM state it can actually be in
 * (e.g. wanted SHOOT, but only SHOOTING once the flywheel is at speed).
 *
 * A subsystem fills in two methods:
 * <ul>
 * <li>{@link #handleStateTransition} -- wanted state (+ sensors) -> system state</li>
 * <li>{@link #applyState} -- system state -> motor outputs</li>
 * </ul>
 *
 * Mechanisms created with {@link #register} are read, simulated and logged
 * automatically, and the states are published under "<Name>/State/...".
 *
 * @param <W> the wanted-state enum
 * @param <S> the system-state enum
 */
public abstract class StateMachineSubsystem<W extends Enum<W>, S extends Enum<S>> extends SubsystemBase
        implements AutoCloseable {
    private final List<Mechanism> mechanisms = new ArrayList<>();
    private W wantedState;
    private S systemState;
    private double stateEnteredTime = Timer.getFPGATimestamp();

    protected StateMachineSubsystem(String name, W initialWantedState, S initialSystemState) {
        super(name);
        this.wantedState = initialWantedState;
        this.systemState = initialSystemState;
    }

    /** Adds a mechanism to this subsystem's update loop. Returns it, so it can be used in a field initializer. */
    protected <M extends Mechanism> M register(M mechanism) {
        mechanisms.add(mechanism);
        return mechanism;
    }

    /** Works out which system state to be in, given what's wanted. Called every loop. */
    protected abstract S handleStateTransition(W wantedState, S currentState);

    /** Sets motor outputs for the current system state. Called every loop. */
    protected abstract void applyState(S systemState);

    /** Runs once each time the system state changes, before applyState. Override to reset timers etc. */
    protected void onStateEntered(S newState, S previousState) {
    }

    /** Extra dashboard values for this subsystem. Override to add your own. */
    protected void logValues() {
    }

    public void setWantedState(W state) {
        wantedState = state;
    }

    /** A command that sets the wanted state once and finishes immediately. */
    public Command setWantedStateCommand(W state) {
        return runOnce(() -> setWantedState(state)).withName(getName() + " -> " + state);
    }

    public W getWantedState() {
        return wantedState;
    }

    public S getSystemState() {
        return systemState;
    }

    /** Seconds since the system state last changed. */
    public double timeInState() {
        return Timer.getFPGATimestamp() - stateEnteredTime;
    }

    /** Applies eco (reduced) current limits to every mechanism in this subsystem. */
    public void setEcoMode(boolean eco) {
        for (Mechanism mechanism : mechanisms) {
            mechanism.setEcoMode(eco);
        }
    }

    public List<Mechanism> getMechanisms() {
        return List.copyOf(mechanisms);
    }

    @Override
    public final void periodic() {
        for (Mechanism mechanism : mechanisms) {
            mechanism.updateInputs();
        }

        S nextState = handleStateTransition(wantedState, systemState);
        if (nextState != systemState) {
            S previous = systemState;
            systemState = nextState;
            stateEnteredTime = Timer.getFPGATimestamp();
            onStateEntered(nextState, previous);
        }
        applyState(systemState);

        String prefix = getName() + "/State/";
        SmartDashboard.putString(prefix + "Wanted", wantedState.toString());
        SmartDashboard.putString(prefix + "System", systemState.toString());
        SmartDashboard.putNumber(prefix + "Time In State", timeInState());
        for (Mechanism mechanism : mechanisms) {
            mechanism.updateTelemetry();
        }
        logValues();
    }

    @Override
    public final void simulationPeriodic() {
        for (Mechanism mechanism : mechanisms) {
            mechanism.simulationPeriodic(0.02);
        }
    }

    /** Frees the motors and CAN IDs (used by unit tests to start fresh). */
    @Override
    public void close() {
        CommandScheduler.getInstance().unregisterSubsystem(this);
        for (Mechanism mechanism : mechanisms) {
            mechanism.close();
        }
    }
}
