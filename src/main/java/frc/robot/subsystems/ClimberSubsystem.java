package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import static frc.robot.Constants.Climber.*;
import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicTorqueCurrentFOC;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;


public class ClimberSubsystem extends SubsystemBase{
    public final TalonFX ClimberMotor = new TalonFX(CLIMBER_MOTOR_ID, new CANBus("canivore"));

    private final MotionMagicTorqueCurrentFOC ClimberRequest = new MotionMagicTorqueCurrentFOC(0);

    public ClimberSubsystem(){
        var ClimberMotorConfigs = new TalonFXConfiguration();

        ClimberMotorConfigs.Slot0.kP = 5;
        ClimberMotorConfigs.Slot0.kI = 0.0;
        ClimberMotorConfigs.Slot0.kD = 0.0;
        ClimberMotorConfigs.Slot0.kS = 0.0;
        ClimberMotorConfigs.Slot0.kV = 0.0;
        ClimberMotorConfigs.Slot0.kA = 0.0;
        ClimberMotorConfigs.MotionMagic.MotionMagicAcceleration = 100;
        ClimberMotorConfigs.MotionMagic.MotionMagicCruiseVelocity = 200;
        ClimberMotorConfigs.MotionMagic.MotionMagicExpo_kA = 0.12;
        ClimberMotorConfigs.MotionMagic.MotionMagicExpo_kV = 0.1;
        ClimberMotorConfigs.MotionMagic.MotionMagicJerk = 0;
        ClimberMotorConfigs.MotorOutput.NeutralMode = NeutralModeValue.Brake;

        ClimberMotor.getConfigurator().apply(ClimberMotorConfigs);
    }

    public void setPosition(double position){
        ClimberMotor.setControl(ClimberRequest.withPosition(position));
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
