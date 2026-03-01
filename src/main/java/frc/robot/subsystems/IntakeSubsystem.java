// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;


import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkBase.PersistMode;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;

public class IntakeSubsystem extends SubsystemBase {

    private final SparkMax roller;

    public IntakeSubsystem() {

        // Crear motor con ID
        roller = new SparkMax(IntakeConstants.ROLLER_MOTOR_ID, MotorType.kBrushless);

        // Configuración
        SparkMaxConfig rollerConfig = new SparkMaxConfig();
        rollerConfig
            .smartCurrentLimit(IntakeConstants.kRollerCurrentLimit)
            .inverted(IntakeConstants.kRollerInverted);

        roller.configure(rollerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    public void runRoller(double speed) {
        roller.set(speed);
    }

    public void stopRoller() {
        roller.set(0);
    }
    
    public void intake() {
    roller.set(IntakeConstants.kIntakeSpeed);
}

public void outtake() {
    roller.set(IntakeConstants.kOuttakeSpeed);
}
    @Override
    public void periodic() {
        // Se ejecuta cada ciclo
    }
}
