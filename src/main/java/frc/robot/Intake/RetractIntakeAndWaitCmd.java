package frc.robot.Intake;

import edu.wpi.first.wpilibj2.command.Command;

public final class RetractIntakeAndWaitCmd extends Command {
    private final IntakeSubsystem intakeSubsystem;

    public RetractIntakeAndWaitCmd(IntakeSubsystem intakeSubsystem) {
        this.intakeSubsystem = intakeSubsystem;
        addRequirements(this.intakeSubsystem);
    }

    @Override
    public void initialize() {
        intakeSubsystem.requestRetractIntake();
    }

    @Override
    public boolean isFinished() {
        return intakeSubsystem.isIntakeRetracted();
    }
}