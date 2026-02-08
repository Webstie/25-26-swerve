package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import static frc.robot.Constants.Shooter.*;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

public class ShooterAngleSystem extends SubsystemBase {
    private final SparkMax shooterAdjustment;
    
    public ShooterAngleSystem() {
        shooterAdjustment = new SparkMax(SHOOTER_SPARKMAX_ID, MotorType.kBrushed);
    }
    public void setShooterAngleVoltage(double Voltage){
        shooterAdjustment.setVoltage(Voltage);
    }
    public Command AdjustShootingAngle(double Voltage){
        return startEnd(
            ()->setShooterAngleVoltage(Voltage),
            ()->setShooterAngleVoltage(0)
        );
    }
}