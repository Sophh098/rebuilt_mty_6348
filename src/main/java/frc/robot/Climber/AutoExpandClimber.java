package frc.robot.Climber;

import edu.wpi.first.wpilibj2.command.Command;

/**
 * Command that automatically expands (extends) the climber until the subsystem reports
 * that the climber is fully extended.
 *
 * Behavior:
 * - On initialize(): requests auto expansion.
 * - On execute(): keeps requesting auto expansion (useful if the subsystem method is "keep driving motor").
 * - On end(): stops the climber motor regardless of whether the command ended normally or was interrupted.
 * - Finishes when: the subsystem reports the climber is extended.
 *
 * Requirements:
 * - Requires {@link ClimberSubsystem}, so no other climber commands can run at the same time.
 */
public final class AutoExpandClimber extends Command {

    /** Subsystem that controls the climber hardware (motor/actuator/sensors). */
    private final ClimberSubsystem climberSubsystem;

    /**
     * Creates a new AutoExpandClimber command.
     *
     * @param climberSubsystem the climber subsystem used to control and read the climber state
     */
    public AutoExpandClimber(ClimberSubsystem climberSubsystem) {
        this.climberSubsystem = climberSubsystem;
        addRequirements(climberSubsystem);
    }

    /**
     * Called once when the command is first scheduled.
     * Starts the automatic expansion behavior.
     */
    @Override
    public void initialize() {
        climberSubsystem.autoExpand();
    }

    /**
     * Called repeatedly while the command is scheduled.
     * Continues commanding expansion until the climber is detected as extended.
     */
    @Override
    public void execute() {
        climberSubsystem.autoExpand();
    }

    /**
     * Called once when the command ends or is interrupted.
     *
     * @param interrupted true if the command was canceled or interrupted by another command
     */
    @Override
    public void end(boolean interrupted) {
        climberSubsystem.stop();
    }

    /**
     * Determines whether the command has completed.
     *
     * @return true when the climber is fully extended according to the subsystem sensor/state
     */
    @Override
    public boolean isFinished() {
        return climberSubsystem.isClimberExtended();
    }
}