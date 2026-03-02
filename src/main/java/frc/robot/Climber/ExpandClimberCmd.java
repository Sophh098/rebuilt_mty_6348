package frc.robot.Climber;

import edu.wpi.first.wpilibj2.command.Command;

public class ExpandClimberCmd extends Command {
    private final ClimberSubsystem climberSubsystem;

    public ExpandClimberCmd(ClimberSubsystem climberSubsystem) {
        this.climberSubsystem = climberSubsystem;
        addRequirements(climberSubsystem);
    }

    @Override
    public void initialize() {
        climberSubsystem.expand();
    }

    @Override
    public boolean isFinished() {
        return true;
    }
    
}
