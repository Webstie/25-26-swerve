package frc.robot.subsystems;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

public class PivotSubsystem extends SubsystemBase {
    private final SparkMax leftActuator = new SparkMax(9, MotorType.kBrushed);

    public void setActuatorVoltage(double voltage) {
        leftActuator.setVoltage(voltage);
    }

    public Command pitchRaiseCommand(double voltage) {
        return startEnd(
            () -> setActuatorVoltage(voltage),
            () -> setActuatorVoltage(0)
        );
    }
}