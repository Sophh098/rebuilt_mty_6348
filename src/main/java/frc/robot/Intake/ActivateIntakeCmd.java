package frc.robot.Intake;

import edu.wpi.first.wpilibj2.command.Command;

public final class ActivateIntakeCmd extends Command {

    private final IntakeSubsystem intakeSubsystem;

    public ActivateIntakeCmd(IntakeSubsystem intakeSubsystem) {
        this.intakeSubsystem = intakeSubsystem;
        addRequirements(this.intakeSubsystem);
    }

    @Override
    public void initialize() {
        intakeSubsystem.setRollerEnabled(true);
    }

    @Override
    public void end(boolean interrupted) {
        intakeSubsystem.setRollerEnabled(false);
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}