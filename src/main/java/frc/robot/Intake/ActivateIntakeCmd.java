package frc.robot.Intake;

import edu.wpi.first.wpilibj2.command.Command;

/**
 * Command that enables the intake roller while the command is scheduled.
 *
 * Behavior:
 * - On initialize(): enables the intake roller.
 * - On end(): disables the intake roller (both on normal end and interruption).
 * - isFinished(): always false, so it will run until canceled/interrupted.
 *
 * Typical usage:
 * - Bind to a controller button with "whileTrue/whileHeld" so the roller runs only while pressed.
 */
public final class ActivateIntakeCmd extends Command {

    /** Subsystem that controls the intake roller hardware. */
    private final IntakeSubsystem intakeSubsystem;

    /**
     * Creates a new ActivateIntakeCmd.
     *
     * @param intakeSubsystem the intake subsystem used to control the roller state
     */
    public ActivateIntakeCmd(IntakeSubsystem intakeSubsystem) {
        this.intakeSubsystem = intakeSubsystem;
        addRequirements(this.intakeSubsystem);
    }

    /**
     * Called once when the command is first scheduled.
     * Enables the intake roller.
     */
    @Override
    public void initialize() {
        intakeSubsystem.setRollerEnabled(true);
    }

    /**
     * Called once when the command ends or is interrupted.
     * Disables the intake roller to ensure the mechanism stops safely.
     *
     * @param interrupted true if the command was canceled or interrupted by another command
     */
    @Override
    public void end(boolean interrupted) {
        intakeSubsystem.setRollerEnabled(false);
    }

    /**
     * This command is intended to be held/canceled externally.
     *
     * @return always false
     */
    @Override
    public boolean isFinished() {
        return false;
    }
}