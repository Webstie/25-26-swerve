package frc.robot.subsystems;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import static frc.robot.Constants.LauncherConfig.*;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkMax;

public class Launcher extends SubsystemBase {

    private double frictionWheelVelocityTarget = 0.0;
    private final SlewRateLimiter velocityLimiter = new SlewRateLimiter(FrictionWheelVelocityRampRate);
    private final NeutralOut Neutral_Request = new NeutralOut();

    private final SparkMax angleAdjustment;

    private final TalonFX IntakeBallMotor = new TalonFX(FEEDER_MOTOR_ID, new CANBus("canivore"));
    private final TalonFX LeftFrictionwheelMotor = new TalonFX(LEFT_FRICTIONWHEEL_MOTOR_ID, new CANBus("canivore"));
    private final TalonFX MiddleFrictionwheelMotor = new TalonFX(MIDDLE_FRICTIONWHEEL_MOTOR_ID, new CANBus("canivore"));
    private final TalonFX RightFrictionwheelMotor = new TalonFX(RIGHT_FRICTIONWHEEL_MOTOR_ID, new CANBus("canivore"));

    private final VelocityTorqueCurrentFOC AllFrictionwheelMotor_Request = new VelocityTorqueCurrentFOC(0.0).withSlot(0);
    private final VelocityTorqueCurrentFOC IntakeBallMotor_Request = new VelocityTorqueCurrentFOC(0.0).withSlot(0);
    
    public Launcher() {
        angleAdjustment = new SparkMax(SHOOTER_SPARKMAX_ID, MotorType.kBrushed);

        var LeftFricwhemotorConfigs = new TalonFXConfiguration();
        LeftFricwhemotorConfigs.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        LeftFricwhemotorConfigs.Slot0.kS = 1.5;
        LeftFricwhemotorConfigs.Slot0.kV = 0.35;
        LeftFricwhemotorConfigs.Slot0.kA = 0;
        LeftFricwhemotorConfigs.Slot0.kP = 8;
        LeftFricwhemotorConfigs.Slot0.kI = 0;
        LeftFricwhemotorConfigs.Slot0.kD = 0;
        LeftFricwhemotorConfigs.MotionMagic.MotionMagicAcceleration = 100; 
        LeftFricwhemotorConfigs.MotionMagic.MotionMagicCruiseVelocity = 200; 
        LeftFricwhemotorConfigs.MotionMagic.MotionMagicExpo_kV = 0.12; 
        LeftFricwhemotorConfigs.MotionMagic.MotionMagicExpo_kA = 0.1; 
        LeftFricwhemotorConfigs.MotionMagic.MotionMagicJerk = 0; 
        LeftFrictionwheelMotor.getConfigurator().apply(LeftFricwhemotorConfigs);

        var MiddleFricwhemotorConfigs = new TalonFXConfiguration();
        MiddleFricwhemotorConfigs.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        MiddleFricwhemotorConfigs.Slot0.kS = 1.5;
        MiddleFricwhemotorConfigs.Slot0.kV = 0.35;
        MiddleFricwhemotorConfigs.Slot0.kA = 0;
        MiddleFricwhemotorConfigs.Slot0.kP = 8;
        MiddleFricwhemotorConfigs.Slot0.kI = 0;
        MiddleFricwhemotorConfigs.Slot0.kD = 0;
        MiddleFricwhemotorConfigs.MotionMagic.MotionMagicAcceleration = 100; 
        MiddleFricwhemotorConfigs.MotionMagic.MotionMagicCruiseVelocity = 200; 
        MiddleFricwhemotorConfigs.MotionMagic.MotionMagicExpo_kV = 0.12; 
        MiddleFricwhemotorConfigs.MotionMagic.MotionMagicExpo_kA = 0.1; 
        MiddleFricwhemotorConfigs.MotionMagic.MotionMagicJerk = 0; 
        MiddleFrictionwheelMotor.getConfigurator().apply(MiddleFricwhemotorConfigs);

        var RightFricwhemotorConfigs = new TalonFXConfiguration();
        RightFricwhemotorConfigs.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        RightFricwhemotorConfigs.Slot0.kS = 1.5;
        RightFricwhemotorConfigs.Slot0.kV = 0.1;
        RightFricwhemotorConfigs.Slot0.kA = 0;
        RightFricwhemotorConfigs.Slot0.kP = 8;
        RightFricwhemotorConfigs.Slot0.kI = 0;
        RightFricwhemotorConfigs.Slot0.kD = 0;
        RightFricwhemotorConfigs.MotionMagic.MotionMagicAcceleration = 100; 
        RightFricwhemotorConfigs.MotionMagic.MotionMagicCruiseVelocity = 200; 
        RightFricwhemotorConfigs.MotionMagic.MotionMagicExpo_kV = 0.12; 
        RightFricwhemotorConfigs.MotionMagic.MotionMagicExpo_kA = 0.1; 
        RightFricwhemotorConfigs.MotionMagic.MotionMagicJerk = 0; 
        RightFrictionwheelMotor.getConfigurator().apply(RightFricwhemotorConfigs);

        var IntakeballmotorConfigs = new TalonFXConfiguration();
        IntakeballmotorConfigs.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        IntakeballmotorConfigs.Slot0.kS = 0.0;
        IntakeballmotorConfigs.Slot0.kV = 0.0;
        IntakeballmotorConfigs.Slot0.kA = 0;
        IntakeballmotorConfigs.Slot0.kP = 5;
        IntakeballmotorConfigs.Slot0.kI = 0;
        IntakeballmotorConfigs.Slot0.kD = 0;
        IntakeballmotorConfigs.MotionMagic.MotionMagicAcceleration = 100;
        IntakeballmotorConfigs.MotionMagic.MotionMagicCruiseVelocity = 200; 
        IntakeballmotorConfigs.MotionMagic.MotionMagicExpo_kV = 0.12; 
        IntakeballmotorConfigs.MotionMagic.MotionMagicExpo_kA = 0.1; 
        IntakeballmotorConfigs.MotionMagic.MotionMagicJerk = 0; 
        IntakeBallMotor.getConfigurator().apply(IntakeballmotorConfigs);
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
    内部应用速度并设置限制函数
     */
    private void applyFrictionWheelVelocity(double Velocity) { 
        LeftFrictionwheelMotor.setControl(AllFrictionwheelMotor_Request.withVelocity(Velocity));
        MiddleFrictionwheelMotor.setControl(AllFrictionwheelMotor_Request.withVelocity(Velocity));
        RightFrictionwheelMotor.setControl(AllFrictionwheelMotor_Request.withVelocity(-Velocity));
    }
    /**
    内部应用温和函数
     */
    private void applyFrictionWheelNeutral() {
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

    /**
    发射机构角度调整单独命令
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
    发射机构进料速度调整接口
     */
    public void setIntakeVelocity(double Velocity) {
        IntakeBallMotor.setControl(IntakeBallMotor_Request.withVelocity(Velocity));
    }

    /**
    发射(摩擦轮+intake)单独命令
     */
    public Command LaunchSingleCommand() { 
        return startEnd(
            () -> { 
                setFrictionWheelVelocity(FrictionWheelLaunchSpeed);
                setIntakeVelocity(IntakeSpeed);
            },
            () -> {
                setFrictionWheelVelocity(0);
                setIntakeVelocity(0);
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
                setIntakeVelocity(OuttakeBallspeed);
            },
            ()->{
                setFrictionWheelVelocity(0.0);
                setIntakeVelocity(0.0);
            }
        );
    }
    
    /**
    摩擦轮预热单独命令
     */
    public Command ShooterWarmupSingleCommand() { 
        return runOnce(
            () -> { 
                setFrictionWheelVelocity(FrictionWheelLaunchSpeed);
            }
        );
    }
}
