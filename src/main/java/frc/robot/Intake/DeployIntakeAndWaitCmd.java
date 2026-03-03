package frc.robot.Intake;

import edu.wpi.first.wpilibj2.command.Command;

/**
 * Command that requests the intake to deploy and then waits until the intake is reported as deployed.
 *
 * Behavior:
 * - On initialize(): sends a deploy request to the subsystem.
 * - execute(): not used (the subsystem is expected to continue acting on the request).
 * - end(): not used (does not automatically retract or stop anything).
 * - Finishes when: {@link IntakeSubsystem#isIntakeDeployed()} returns true.
 *
 * Typical usage:
 * - As a step in an autonomous sequence: deploy intake, then run rollers, then drive, etc.
 */
public final class DeployIntakeAndWaitCmd extends Command {

    /** Subsystem responsible for intake deployment state and actuation. */
    private final IntakeSubsystem intakeSubsystem;

    /**
     * Creates a new DeployIntakeAndWaitCmd.
     *
     * @param intakeSubsystem the intake subsystem that will handle the deploy request
     */
    public DeployIntakeAndWaitCmd(IntakeSubsystem intakeSubsystem) {
        this.intakeSubsystem = intakeSubsystem;
        addRequirements(this.intakeSubsystem);
    }

    /**
     * Called once when the command is first scheduled.
     * Requests that the intake deploy.
     */
    @Override
    public void initialize() {
        intakeSubsystem.requestDeployIntake();
    }

    /**
     * Determines whether the command has completed.
     *
     * @return true when the intake is confirmed deployed by the subsystem/sensor/state
     */
    @Override
    public boolean isFinished() {
        return intakeSubsystem.isIntakeDeployed();
    }
}