package frc.robot.Climber;

import edu.wpi.first.wpilibj2.command.Command;

public final class AutoExpandClimber extends Command {

    private final ClimberSubsystem climberSubsystem;

    public AutoExpandClimber(ClimberSubsystem climberSubsystem) {
        this.climberSubsystem = climberSubsystem;
        addRequirements(climberSubsystem);
    }

    @Override
    public void initialize() {
        climberSubsystem.autoExpand();
    }

    @Override
    public void execute() {
        climberSubsystem.autoExpand();
    }

    @Override
    public void end(boolean interrupted) {
        climberSubsystem.stop();
    }

    @Override
    public boolean isFinished() {
        return climberSubsystem.isClimberExtended();
    }
}