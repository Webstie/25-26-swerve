package frc.robot.subsystems;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import static frc.robot.Constants.LauncherConfig.*;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;


public class Launcher extends SubsystemBase {

    private static final double ANGLE_TOLERANCE = 0.001;

    private double frictionWheelVelocityTarget = 0.0;
    private final SlewRateLimiter velocityLimiter = new SlewRateLimiter(FrictionWheelVelocityRampRate);
    private final NeutralOut Neutral_Request = new NeutralOut();

    private final TalonFX FeederMotor = new TalonFX(FEEDER_MOTOR_ID, new CANBus("canivore"));
    private final TalonFX LeftFrictionwheelMotor = new TalonFX(LEFT_FRICTIONWHEEL_MOTOR_ID, new CANBus("canivore"));
    private final TalonFX MiddleFrictionwheelMotor = new TalonFX(MIDDLE_FRICTIONWHEEL_MOTOR_ID, new CANBus("canivore"));
    private final TalonFX RightFrictionwheelMotor = new TalonFX(RIGHT_FRICTIONWHEEL_MOTOR_ID, new CANBus("canivore"));
    private final SparkMax angleAdjustment;
    private final CANcoder angleEncoder;
    
    private final VelocityTorqueCurrentFOC AllFrictionwheelMotor_Request = new VelocityTorqueCurrentFOC(0.0).withSlot(0);
    private final VelocityTorqueCurrentFOC FeederMotor_Request = new VelocityTorqueCurrentFOC(0.0).withSlot(0);
    
    public Launcher() {
        angleAdjustment = new SparkMax(SHOOTER_SPARKMAX_ID, MotorType.kBrushed);
        angleEncoder = new CANcoder(ANGLE_CANCODER_ID, "canivore");
        
        var LeftFricwhemotorConfigs = new TalonFXConfiguration();
        LeftFricwhemotorConfigs.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        LeftFricwhemotorConfigs.Slot0.kS = 1.5;
        LeftFricwhemotorConfigs.Slot0.kV = 0.15;
        LeftFricwhemotorConfigs.Slot0.kA = 0;
        LeftFricwhemotorConfigs.Slot0.kP = 6.5;
        LeftFricwhemotorConfigs.Slot0.kI = 0;
        LeftFricwhemotorConfigs.Slot0.kD = 0.2;
        LeftFricwhemotorConfigs.MotionMagic.MotionMagicAcceleration = 100; 
        LeftFricwhemotorConfigs.MotionMagic.MotionMagicCruiseVelocity = 200; 
        LeftFricwhemotorConfigs.MotionMagic.MotionMagicExpo_kV = 0.12; 
        LeftFricwhemotorConfigs.MotionMagic.MotionMagicExpo_kA = 0.1; 
        LeftFricwhemotorConfigs.MotionMagic.MotionMagicJerk = 0; 
        LeftFrictionwheelMotor.getConfigurator().apply(LeftFricwhemotorConfigs);

        var MiddleFricwhemotorConfigs = new TalonFXConfiguration();
        MiddleFricwhemotorConfigs.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        MiddleFricwhemotorConfigs.Slot0.kS = 1.5;
        MiddleFricwhemotorConfigs.Slot0.kV = 0.15;
        MiddleFricwhemotorConfigs.Slot0.kA = 0;
        MiddleFricwhemotorConfigs.Slot0.kP = 6.5;
        MiddleFricwhemotorConfigs.Slot0.kI = 0;
        MiddleFricwhemotorConfigs.Slot0.kD = 0.2;
        MiddleFricwhemotorConfigs.MotionMagic.MotionMagicAcceleration = 100; 
        MiddleFricwhemotorConfigs.MotionMagic.MotionMagicCruiseVelocity = 200; 
        MiddleFricwhemotorConfigs.MotionMagic.MotionMagicExpo_kV = 0.12; 
        MiddleFricwhemotorConfigs.MotionMagic.MotionMagicExpo_kA = 0.1; 
        MiddleFricwhemotorConfigs.MotionMagic.MotionMagicJerk = 0; 
        MiddleFrictionwheelMotor.getConfigurator().apply(MiddleFricwhemotorConfigs);

        var RightFricwhemotorConfigs = new TalonFXConfiguration();
        RightFricwhemotorConfigs.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        RightFricwhemotorConfigs.Slot0.kS = 1.5;
        RightFricwhemotorConfigs.Slot0.kV = 0.15;
        RightFricwhemotorConfigs.Slot0.kA = 0;
        RightFricwhemotorConfigs.Slot0.kP = 6.5;
        RightFricwhemotorConfigs.Slot0.kI = 0;
        RightFricwhemotorConfigs.Slot0.kD = 0.2;
        RightFricwhemotorConfigs.MotionMagic.MotionMagicAcceleration = 100; 
        RightFricwhemotorConfigs.MotionMagic.MotionMagicCruiseVelocity = 200; 
        RightFricwhemotorConfigs.MotionMagic.MotionMagicExpo_kV = 0.12; 
        RightFricwhemotorConfigs.MotionMagic.MotionMagicExpo_kA = 0.1; 
        RightFricwhemotorConfigs.MotionMagic.MotionMagicJerk = 0; 
        RightFrictionwheelMotor.getConfigurator().apply(RightFricwhemotorConfigs);

        var FeederMotorConfigs = new TalonFXConfiguration();
        FeederMotorConfigs.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        FeederMotorConfigs.Slot0.kS = 0.0;
        FeederMotorConfigs.Slot0.kV = 0.0;
        FeederMotorConfigs.Slot0.kA = 0;
        FeederMotorConfigs.Slot0.kP = 5;
        FeederMotorConfigs.Slot0.kI = 0;
        FeederMotorConfigs.Slot0.kD = 0;
        FeederMotorConfigs.MotionMagic.MotionMagicAcceleration = 100;
        FeederMotorConfigs.MotionMagic.MotionMagicCruiseVelocity = 200; 
        FeederMotorConfigs.MotionMagic.MotionMagicExpo_kV = 0.12; 
        FeederMotorConfigs.MotionMagic.MotionMagicExpo_kA = 0.1; 
        FeederMotorConfigs.MotionMagic.MotionMagicJerk = 0; 
        FeederMotor.getConfigurator().apply(FeederMotorConfigs);
    }

    @Override
    public void periodic() {
        if (frictionWheelVelocityTarget == 0.0) {
            applyFrictionWheelNeutral();
        } else {
            double limitedVelocity = velocityLimiter.calculate(frictionWheelVelocityTarget);
            applyFrictionWheelVelocity(limitedVelocity);
        }
    }

    /**
    设置速度
     */
    private void applyFrictionWheelVelocity(double Velocity) { 
        LeftFrictionwheelMotor.setControl(AllFrictionwheelMotor_Request.withVelocity(Velocity));
        MiddleFrictionwheelMotor.setControl(AllFrictionwheelMotor_Request.withVelocity(Velocity));
        RightFrictionwheelMotor.setControl(AllFrictionwheelMotor_Request.withVelocity(-Velocity));
    }
    /**
    缓慢释放电机
     */
    public void applyFrictionWheelNeutral() {
        LeftFrictionwheelMotor.setControl(Neutral_Request);
        MiddleFrictionwheelMotor.setControl(Neutral_Request);
        RightFrictionwheelMotor.setControl(Neutral_Request);
    }

    /**
    发射机构角度调整接口
     */
    public void setAngleVoltage(double Voltage){
        angleAdjustment.setVoltage(Voltage);
    }

    public void setAngleToTarget(double targetPosition){
        double error = angleEncoder.getAbsolutePosition().getValueAsDouble() - targetPosition;
        if (Math.abs(error) <= ANGLE_TOLERANCE) {
            setAngleVoltage(0);
            return;
        }
        setAngleVoltage(error >= 0 ? -12 : 12);
    }

    public Command AdjustAngleToPositionCommand(double targetPosition){
        final double kP = 200.0;
        final double maxVoltage = 12.0;
        
        // 降低基础克服摩擦力的电压（kS）。8.0V太暴力了，2.0~3.0V通常足够让推杆缓慢移动
        final double kS = 5.0; 

        return run(
            () -> {
                double current = angleEncoder.getAbsolutePosition().getValueAsDouble();
                double error = targetPosition - current;
                double absError = Math.abs(error);

                // 1. 如果已经进入容差范围，直接输出 0 并返回
                if (absError <= ANGLE_TOLERANCE) {
                    setAngleVoltage(0);
                    return;
                }

                // 2. 计算比例输出 (P)
                double pOutput = kP * error;
                
                // 3. 摩擦力前馈补偿 (带有方向的最小启动电压)
                // 只要有误差，我们就给它一个基础力 kS，再加上随误差变小的 P 力
                double feedforward = Math.copySign(kS, error);
                
                // 4. 总输出 = P输出 + 前馈补偿，并限制在最大电压内
                double voltage = MathUtil.clamp(pOutput + feedforward, -maxVoltage, maxVoltage);
                
                setAngleVoltage(voltage);
            }
        )
        // 结束条件：绝对误差小于容差
        .until(() -> Math.abs(targetPosition - angleEncoder.getAbsolutePosition().getValueAsDouble()) <= ANGLE_TOLERANCE)
        
        // 🌟 核心保命机制：最多只给它 1.5 秒的时间调整！🌟
        // 如果推杆发生机械卡死，或者由于某种原因还在轻微震荡，1.5秒一到强制宣告调整结束，进入下一步发射！
        .withTimeout(1.5)
        
        // 无论是正常达到位置，还是超时被打断，都确保电机断电
        .finallyDo(() -> setAngleVoltage(0));
    }

    /**
    发射机构电推杆角度调整单独命令
     */
    public Command AdjustAngleSingleCommand(double Voltage){
        return startEnd(
            ()->setAngleVoltage(Voltage),
            ()->setAngleVoltage(0)
        );
    }

    /**
    发射机构摩擦轮速度调整接口
     */
    public void setFrictionWheelVelocity(double Velocity) { 
        frictionWheelVelocityTarget = Velocity;
    }


    /**
    发射机构feeder速度调整接口
     */
    public void setFeederVelocity(double Velocity) {
        FeederMotor.setControl(FeederMotor_Request.withVelocity(Velocity));
    }

    /**
    发射(摩擦轮+intake)单独命令
     */
    public Command LaunchSingleCommand(double FrictionWheelLaunchSpeed) { 
        return startEnd(
            () -> { 
                setFrictionWheelVelocity(FrictionWheelLaunchSpeed);
                setFeederVelocity(FeederSpeed);
            },
            () -> {
                setFrictionWheelVelocity(0);
                setFeederVelocity(0);
            }
        );
    }

    /**
    Outtake单独命令（摩擦轮+intake反转）
     */
    public Command OuttakeSingleCommand(){
        return startEnd(
            ()->{
                setFrictionWheelVelocity(-40.0);
                setFeederVelocity(OuttakeBallspeed);
            },
            ()->{
                setFrictionWheelVelocity(0.0);
                setFeederVelocity(0.0);
            }
        );
    }
    
    /**
    摩擦轮预热单独命令
     */
    public Command ShooterWarmupSingleCommand(double FrictionWheelLaunchSpeed) { 
        return runOnce(
            () -> { 
                setFrictionWheelVelocity(FrictionWheelLaunchSpeed);
            }
        );
    }
}

