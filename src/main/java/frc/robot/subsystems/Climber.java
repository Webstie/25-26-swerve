package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.MathUtil;
import static frc.robot.Constants.ClimberConfig.*;
import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicTorqueCurrentFOC;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;


public class Climber extends SubsystemBase{
    public final TalonFX ClimberMotor = new TalonFX(CLIMBER_MOTOR_ID, new CANBus("canivore"));

    private final MotionMagicVoltage ClimberRequest = new MotionMagicVoltage(0).withSlot(0);
    private final VoltageOut ZeroVoltageRequest = new VoltageOut(0.0);
    private static final double POSITION_TOLERANCE = 1.0;


    public Climber(){
        var ClimberMotorConfigs = new TalonFXConfiguration();

        ClimberMotorConfigs.Slot0.kP = 10;
        ClimberMotorConfigs.Slot0.kI = 0.0;
        ClimberMotorConfigs.Slot0.kD = 0.0;
        ClimberMotorConfigs.Slot0.kS = 0.5;
        ClimberMotorConfigs.Slot0.kV = 0.5;
        ClimberMotorConfigs.Slot0.kA = 0.0;
        ClimberMotorConfigs.MotionMagic.MotionMagicAcceleration = 1000;
        ClimberMotorConfigs.MotionMagic.MotionMagicCruiseVelocity = 50;
        ClimberMotorConfigs.MotionMagic.MotionMagicExpo_kA = 0.12;
        ClimberMotorConfigs.MotionMagic.MotionMagicExpo_kV = 0.1;
        ClimberMotorConfigs.MotionMagic.MotionMagicJerk = 0;
        ClimberMotorConfigs.MotorOutput.NeutralMode = NeutralModeValue.Brake;

        ClimberMotor.getConfigurator().apply(ClimberMotorConfigs);
    }

    public void releaseClimber() {
        ClimberMotor.setControl(ZeroVoltageRequest);
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

    public boolean isAtPosition(double targetPosition){
        return Math.abs(getCurrentPosition() - targetPosition) < POSITION_TOLERANCE;
    }
    
    /**
    攀爬流程单独命令
     */
    public Command ClimbingProcessSingleCommand(){
        return runEnd(
                () -> setPosition(ClimberTopPosition), 
                () -> setPosition(getCurrentPosition())
            ).until(() -> isAtPosition(ClimberTopPosition));
    }

        public Command ClimbingDownSingleCommand(){
            return runEnd(
                () -> setPosition(ClimbPosition), 
                () -> setPosition(getCurrentPosition())
            ).until(() -> isAtPosition(ClimbPosition));
    }

    /**
    攀爬开始单独命令
     */
    public Command ClimbSingleCommand(){
        return run(
                () -> setPosition(ClimbPosition)
            ).until(() -> isAtPosition(ClimbPosition));
    }
}
