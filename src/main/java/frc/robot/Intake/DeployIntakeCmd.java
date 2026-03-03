package frc.robot.Intake;

import edu.wpi.first.wpilibj2.command.Command;

/**
 * Instant command that requests the intake to deploy.
 *
 * Behavior:
 * - On initialize(): calls {@link IntakeSubsystem#requestDeployIntake()}.
 * - Immediately finishes (isFinished() returns true).
 *
 * Important behavior note:
 * - This command does not wait for deployment to complete.
 *   If you need to block until the intake is actually deployed, use a "deploy and wait" command
 *   (like your {@code DeployIntakeAndWaitCmd}).
 *
 * Typical usage:
 * - Trigger deploy on a button press, while something else (subsystem logic or another command)
 *   manages the motion until it reaches the deployed state.
 */
public final class DeployIntakeCmd extends Command {

    /** Subsystem responsible for intake deployment state and actuation. */
    private final IntakeSubsystem intakeSubsystem;

    /**
     * Creates a new DeployIntakeCmd.
     *
     * @param intakeSubsystem the intake subsystem that will receive the deploy request
     */
    public DeployIntakeCmd(IntakeSubsystem intakeSubsystem) {
        this.intakeSubsystem = intakeSubsystem;
        addRequirements(this.intakeSubsystem);
    }

    /**
     * Called once when the command is first scheduled.
     * Requests intake deployment.
     */
    @Override
    public void initialize() {
        intakeSubsystem.requestDeployIntake();
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