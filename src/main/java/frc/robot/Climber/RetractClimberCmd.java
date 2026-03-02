package frc.robot.Climber;

import edu.wpi.first.wpilibj2.command.Command;

public class RetractClimberCmd extends Command {
    private final ClimberSubsystem climberSubsystem;

    public RetractClimberCmd(ClimberSubsystem climberSubsystem) {
        this.climberSubsystem = climberSubsystem;
        addRequirements(climberSubsystem);
    }

    @Override
    public void initialize() {
        climberSubsystem.retract();
    }

    @Override
    public boolean isFinished() {
        return true;
    }
    
}
