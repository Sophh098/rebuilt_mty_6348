package frc.robot.Climber;

import edu.wpi.first.wpilibj2.command.Command;

/**
 * Instant command that requests climber expansion using manual (open-loop) voltage.
 *
 * What it does:
 * - On initialize(): calls {@link ClimberSubsystem#expand()} to set MANUAL_VOLTAGE mode and request expansion voltage.
 * - Immediately finishes (isFinished() returns true).
 *
 * Important behavior note:
 * - Because this command ends immediately, it does NOT keep commanding expansion.
 *   The climber will only continue moving if the subsystem's periodic() keeps applying the last requested voltage
 *   (which it currently does in MANUAL_VOLTAGE mode), and nothing else changes the requested voltage afterward.
 *
 * Use cases:
 * - Useful as a "one-shot" action to start expansion when you want another system/state machine to decide when to stop.
 */
public class ExpandClimberCmd extends Command {

    /** Subsystem that controls the climber hardware. */
    private final ClimberSubsystem climberSubsystem;

    /**
     * Creates a new ExpandClimberCmd.
     *
     * @param climberSubsystem the climber subsystem used to command manual expansion
     */
    public ExpandClimberCmd(ClimberSubsystem climberSubsystem) {
        this.climberSubsystem = climberSubsystem;
        addRequirements(climberSubsystem);
    }

    /**
     * Called once when the command is first scheduled.
     * Requests manual expansion.
     */
    @Override
    public void initialize() {
        climberSubsystem.expand();
    }

    /**
     * This is an instant command: it ends immediately after initialize().
     *
     * @return always true
     */
    @Override
    public boolean isFinished() {
        return true;
    }
}