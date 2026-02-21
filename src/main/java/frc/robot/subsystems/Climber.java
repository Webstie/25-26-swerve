package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import static frc.robot.Constants.ClimberConfig.*;
import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicTorqueCurrentFOC;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;


public class Climber extends SubsystemBase{
    public final TalonFX ClimberMotor = new TalonFX(CLIMBER_MOTOR_ID, new CANBus("canivore"));

    private final MotionMagicTorqueCurrentFOC ClimberRequest = new MotionMagicTorqueCurrentFOC(0);

    public Climber(){
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

    /**
    攀爬位置设置接口
     */
    public void setPosition(double position){
        ClimberMotor.setControl(ClimberRequest.withPosition(position));
    }

    /**
    攀爬位置获取接口
     */
    public double getCurrentPosition(){
        return ClimberMotor.getPosition().getValueAsDouble();
    }
    
    /**
    攀爬流程单独命令
     */
    public Command ClimbingProcessSingleCommand(){
        return runEnd(
                () -> setPosition(ClimberTopPosition), 
                () -> setPosition(getCurrentPosition())
            ).until(() -> (Math.abs(getCurrentPosition() - ClimberTopPosition) < 1.0));
    }

    /**
    攀爬开始单独命令
     */
    public Command ClimbSingleCommand(){
        return runOnce(() -> setPosition(ClimbPosition));
    }
}
