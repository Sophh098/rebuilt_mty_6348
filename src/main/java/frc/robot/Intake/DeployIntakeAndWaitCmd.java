package frc.robot.Intake;

import edu.wpi.first.wpilibj2.command.Command;

public final class DeployIntakeAndWaitCmd extends Command {
    private final IntakeSubsystem intakeSubsystem;

    public DeployIntakeAndWaitCmd(IntakeSubsystem intakeSubsystem) {
        this.intakeSubsystem = intakeSubsystem;
        addRequirements(this.intakeSubsystem);
    }

    @Override
    public void initialize() {
        intakeSubsystem.requestDeployIntake();
    }

    @Override
    public boolean isFinished() {
        return intakeSubsystem.isIntakeDeployed();
    }
}