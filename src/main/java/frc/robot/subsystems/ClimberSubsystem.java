package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.Climber;
import static frc.robot.Constants.Climber.*;

import static edu.wpi.first.units.Units.Rotation;

import java.io.File;
import java.util.function.Supplier;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicTorqueCurrentFOC;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.motorcontrol.Talon;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;

public class ClimberSubsystem extends SubsystemBase{
    public final TalonFX ClimberMotor = new TalonFX(CLIMBER_MOTOR_ID, "rio");


    private final MotionMagicTorqueCurrentFOC CLimberRequest = new MotionMagicTorqueCurrentFOC(0);


    public ClimberSubsystem(){
        var ClimberMotorConfigs = new TalonFXConfiguration();

        ClimberMotorConfigs.Slot0.kP = 4.80;
        ClimberMotorConfigs.Slot0.kI = 0.0;
        ClimberMotorConfigs.Slot0.kD = 0.1;
        ClimberMotorConfigs.Slot0.kS = 0.25;
        ClimberMotorConfigs.Slot0.kV = 0.12;
        ClimberMotorConfigs.Slot0.kA = 0.01;
        ClimberMotorConfigs.MotionMagic.MotionMagicAcceleration = 100;
        ClimberMotorConfigs.MotionMagic.MotionMagicCruiseVelocity = 200;
        ClimberMotorConfigs.MotionMagic.MotionMagicExpo_kA = 0.12;
        ClimberMotorConfigs.MotionMagic.MotionMagicExpo_kV = 0.1;
        ClimberMotorConfigs.MotionMagic.MotionMagicJerk = 0;
        ClimberMotorConfigs.MotorOutput.NeutralMode = NeutralModeValue.Brake;

        ClimberMotor.getConfigurator().apply(ClimberMotorConfigs);
    }

    public void setPosition(double position){
        ClimberMotor.setControl(CLimberRequest.withPosition(position));
    }
    public double getCurrentPosition(){return ClimberMotor.getPosition().getValueAsDouble();}
    
    public Command StartClimb(){
        return runEnd(
                () -> setPosition(ClimberTopPosition), 
                () -> setPosition(getCurrentPosition())
            ).until(() -> (Math.abs(getCurrentPosition() - ClimberTopPosition) < 1.0));
    }
    public Command Climb(){
        return runOnce(() -> setPosition(ClimbPosition));
    }

}
