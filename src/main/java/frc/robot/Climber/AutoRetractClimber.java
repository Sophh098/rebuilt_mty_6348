package frc.robot.Climber;

import edu.wpi.first.wpilibj2.command.Command;

public final class AutoRetractClimber extends Command {

    private final ClimberSubsystem climberSubsystem;

    public AutoRetractClimber(ClimberSubsystem climberSubsystem) {
        this.climberSubsystem = climberSubsystem;
        addRequirements(climberSubsystem);
    }

    @Override
    public void initialize() {
        climberSubsystem.autoRetract();
    }

    @Override
    public void execute() {
        climberSubsystem.autoRetract();
    }

    @Override
    public void end(boolean interrupted) {
        climberSubsystem.stop();
    }

    @Override
    public boolean isFinished() {
        return climberSubsystem.isClimberRetracted();
    }
}