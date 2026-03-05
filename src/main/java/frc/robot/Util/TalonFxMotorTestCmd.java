package frc.robot.Util;
import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Drive.generated.TunerConstants;

public class TalonFxMotorTestCmd extends Command {

    private final TalonFX motorController;
    private final DoubleSupplier motorSpeedSupplier;
    private final boolean isMotorReversed;

    private final DutyCycleOut dutyCycleRequest = new DutyCycleOut(0);

    private final int motorIdentifier;
    private final String motorName;
    private final String dashboardKeyPrefix;
    private final double maxDuty;

    public TalonFxMotorTestCmd(
        String motorName,
        int motorIdentifier,
        DoubleSupplier speedSupplier,
        boolean isReversed,
        double maxDuty
    ) {

        this.motorName = motorName;
        this.motorIdentifier = motorIdentifier;
        this.maxDuty = maxDuty;
        this.motorController = new TalonFX(
            motorIdentifier,
            TunerConstants.CONTROLLER_AREA_NETWORK_BUS
        );

        this.motorSpeedSupplier = speedSupplier;
        this.isMotorReversed = isReversed;

        this.dashboardKeyPrefix =
            "MotorTest/" + motorName + "(" + motorIdentifier + ")/";
    }

    @Override
    public void execute() {

        double requestedSpeed = motorSpeedSupplier.getAsDouble() * maxDuty;

        if (Math.abs(requestedSpeed) < 0.01) {
            requestedSpeed = 0.0;
        }

        if (isMotorReversed) {
            requestedSpeed = -requestedSpeed;
        }

        motorController.setControl(
            dutyCycleRequest.withOutput(requestedSpeed)
        );

        SmartDashboard.putNumber(
            dashboardKeyPrefix + "RequestedOutput",
            requestedSpeed
        );

        SmartDashboard.putNumber(
            dashboardKeyPrefix + "PositionRotations",
            motorController.getPosition().getValueAsDouble()
        );

        SmartDashboard.putNumber(
            dashboardKeyPrefix + "VelocityRps",
            motorController.getVelocity().getValueAsDouble()
        );

        SmartDashboard.putNumber(
            dashboardKeyPrefix + "SupplyCurrentAmps",
            motorController.getSupplyCurrent().getValueAsDouble()
        );

        SmartDashboard.putNumber(
            dashboardKeyPrefix + "StatorCurrentAmps",
            motorController.getStatorCurrent().getValueAsDouble()
        );

        SmartDashboard.putNumber(
            dashboardKeyPrefix + "DeviceTempCelsius",
            motorController.getDeviceTemp().getValueAsDouble()
        );
    }

    @Override
    public void end(boolean interrupted) {
        motorController.setControl(dutyCycleRequest.withOutput(0.0));

        SmartDashboard.putBoolean(
            dashboardKeyPrefix + "Stopped",
            true
        );
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}