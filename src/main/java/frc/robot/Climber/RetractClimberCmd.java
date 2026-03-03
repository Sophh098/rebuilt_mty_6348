package frc.robot.Climber;

import edu.wpi.first.wpilibj2.command.Command;

/**
 * Instant command that requests climber retraction using manual (open-loop) voltage.
 *
 * What it does:
 * - On initialize(): calls {@link ClimberSubsystem#retract()} to set MANUAL_VOLTAGE mode and request retraction voltage.
 * - Immediately finishes (isFinished() returns true).
 *
 * Important behavior note:
 * - Because this command ends immediately, it does NOT keep commanding retraction itself.
 *   Retraction will continue only if the subsystem's periodic() keeps applying the last requested voltage
 *   (which it currently does in MANUAL_VOLTAGE mode) and no other command changes that request afterward.
 *
 * Use cases:
 * - Useful as a one-shot to start retracting when something else decides when/how to stop.
 */
public class RetractClimberCmd extends Command {

    /** Subsystem that controls the climber hardware. */
    private final ClimberSubsystem climberSubsystem;

    /**
     * Creates a new RetractClimberCmd.
     *
     * @param climberSubsystem the climber subsystem used to command manual retraction
     */
    public RetractClimberCmd(ClimberSubsystem climberSubsystem) {
        this.climberSubsystem = climberSubsystem;
        addRequirements(climberSubsystem);
    }

    /**
     * Called once when the command is first scheduled.
     * Requests manual retraction.
     */
    @Override
    public void initialize() {
        climberSubsystem.retract();
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