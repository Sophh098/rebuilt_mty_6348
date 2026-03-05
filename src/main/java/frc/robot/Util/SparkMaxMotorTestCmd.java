package frc.robot.Util;

import java.util.function.DoubleSupplier;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

public class SparkMaxMotorTestCmd extends Command {

    private final SparkMax motorController;
    private final RelativeEncoder motorEncoder;

    private final DoubleSupplier motorSpeedSupplier;
    private final boolean isMotorReversed;

    private final int motorIdentifier;
    private final String motorName;
    private final String dashboardKeyPrefix;
    private final double maxDuty;

    public SparkMaxMotorTestCmd(
        String motorName,
        int motorIdentifier,
        DoubleSupplier speedSupplier,
        boolean isReversed,
        double maxDuty
    ) {

        this.motorName = motorName;
        this.motorIdentifier = motorIdentifier;

        this.motorController = new SparkMax(
            motorIdentifier,
            MotorType.kBrushless
        );

        this.motorEncoder = motorController.getEncoder();

        this.motorSpeedSupplier = speedSupplier;
        this.isMotorReversed = isReversed;

        this.dashboardKeyPrefix =
            "MotorTest/" + motorName + "(" + motorIdentifier + ")/";
        this.maxDuty = maxDuty;
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

        motorController.set(requestedSpeed);

        SmartDashboard.putNumber(
            dashboardKeyPrefix + "RequestedOutput",
            requestedSpeed
        );

        SmartDashboard.putNumber(
            dashboardKeyPrefix + "PositionRotations",
            motorEncoder.getPosition()
        );

        SmartDashboard.putNumber(
            dashboardKeyPrefix + "VelocityRpm",
            motorEncoder.getVelocity()
        );

        SmartDashboard.putNumber(
            dashboardKeyPrefix + "OutputCurrentAmps",
            motorController.getOutputCurrent()
        );

        SmartDashboard.putNumber(
            dashboardKeyPrefix + "AppliedOutput",
            motorController.getAppliedOutput()
        );

        SmartDashboard.putNumber(
            dashboardKeyPrefix + "MotorTemperatureCelsius",
            motorController.getMotorTemperature()
        );

        SmartDashboard.putNumber(dashboardKeyPrefix + "RawAxis", motorSpeedSupplier.getAsDouble());
    }

    @Override
    public void end(boolean interrupted) {

        motorController.set(0.0);

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