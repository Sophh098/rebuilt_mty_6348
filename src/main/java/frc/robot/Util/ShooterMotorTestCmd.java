package frc.robot.Util;

import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.HoodConstants;
import frc.robot.Drive.generated.TunerConstants;

public class ShooterMotorTestCmd extends Command {

    private final TalonFX rightWheel;
    private final TalonFX leftWheel;
    private final TalonFX indexer;

    private final DutyCycleOut rightWheelRequest = new DutyCycleOut(0.0);
    private final DutyCycleOut leftWheelRequest = new DutyCycleOut(0.0);
    private final DutyCycleOut indexerRequest = new DutyCycleOut(0.0);

    private final double rightWheelOutput;
    private final double leftWheelOutput;
    private final double indexerOutput;

    public ShooterMotorTestCmd(
        double rightWheelOutput,
        double leftWheelOutput,
        double indexerOutput
    ) {
        this.rightWheel = new TalonFX(HoodConstants.RIGHT_HOOD_PROPULSION_TALON_ID, TunerConstants.CONTROLLER_AREA_NETWORK_BUS);
        this.leftWheel = new TalonFX(HoodConstants.LEFT_HOOD_PROPULSION_TALON_ID, TunerConstants.CONTROLLER_AREA_NETWORK_BUS);
        this.indexer = new TalonFX(HoodConstants.INDEXER_TALON_ID, TunerConstants.CONTROLLER_AREA_NETWORK_BUS);

        this.rightWheelOutput = rightWheelOutput;
        this.leftWheelOutput = leftWheelOutput;
        this.indexerOutput = indexerOutput;

    }

    @Override
    public void initialize() {
        stopAllMotors();
    }

    @Override
    public void execute() {
        rightWheel.setControl(rightWheelRequest.withOutput(rightWheelOutput));
        leftWheel.setControl(leftWheelRequest.withOutput(leftWheelOutput));
        indexer.setControl(indexerRequest.withOutput(indexerOutput));
    }

    @Override
    public void end(boolean interrupted) {
        stopAllMotors();
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    private void stopAllMotors() {
        rightWheel.setControl(rightWheelRequest.withOutput(0.0));
        leftWheel.setControl(leftWheelRequest.withOutput(0.0));
        indexer.setControl(indexerRequest.withOutput(0.0));
    }
}