package frc.robot.Climber;

import edu.wpi.first.wpilibj2.command.Command;

/**
 * Command that automatically retracts the climber until the subsystem reports
 * that the climber is fully retracted.
 *
 * Behavior:
 * - On initialize(): requests auto retraction.
 * - On execute(): keeps requesting auto retraction (useful if the subsystem method actively drives hardware).
 * - On end(): stops the climber motor regardless of normal end or interruption.
 * - Finishes when: the subsystem reports the climber is retracted.
 *
 * Requirements:
 * - Requires {@link ClimberSubsystem}, preventing other climber commands from running simultaneously.
 */
public final class AutoRetractClimber extends Command {

    /** Subsystem that controls the climber hardware (motor/actuator/sensors). */
    private final ClimberSubsystem climberSubsystem;

    /**
     * Creates a new AutoRetractClimber command.
     *
     * @param climberSubsystem the climber subsystem used to control and read the climber state
     */
    public AutoRetractClimber(ClimberSubsystem climberSubsystem) {
        this.climberSubsystem = climberSubsystem;
        addRequirements(climberSubsystem);
    }

    /**
     * Called once when the command is first scheduled.
     * Starts the automatic retraction behavior.
     */
    @Override
    public void initialize() {
        climberSubsystem.autoRetract();
    }

    /**
     * Called repeatedly while the command is scheduled.
     * Continues commanding retraction until the climber is detected as retracted.
     */
    @Override
    public void execute() {
        climberSubsystem.autoRetract();
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
     * @return true when the climber is fully retracted according to the subsystem sensor/state
     */
    @Override
    public boolean isFinished() {
        return climberSubsystem.isClimberRetracted();
    }
}