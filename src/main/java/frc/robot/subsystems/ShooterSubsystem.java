package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import static frc.robot.Constants.Shooter.*;

public class ShooterSubsystem extends SubsystemBase {//变量写在这里

    private final TalonFX IntakeBallMotor = new TalonFX(IntakeBallMotorID,"rio");
    private final TalonFX LeftFrictionwheelMotor = new TalonFX(LeftFrictionwheelMotorID,"rio");
    private final TalonFX MiddleFrictionwheelMotor = new TalonFX(MiddleFrictionwheelMotorID,"rio");
    private final TalonFX RightFrictionwheelMotor = new TalonFX(RightFrictionwheelMotorID,"rio");
    
   //声明控制请求 还没有实际和电机发生关系
   
   private final VelocityTorqueCurrentFOC AllFrictionwheelMotor_Request = new VelocityTorqueCurrentFOC(0.0).withSlot(0);
   private final VelocityTorqueCurrentFOC IntakeBallMotor_Request = new VelocityTorqueCurrentFOC(0.0).withSlot(0);
  
    //储存某个按键摁下次数
    private int Frictionwhepressed_times = 1;
    private int Intakeballpressed_times = 1;


    //方法1：构造函数
    public ShooterSubsystem() {

        var LeftFricwhemotorConfigs = new TalonFXConfiguration();
        LeftFricwhemotorConfigs.Slot0.kS = 1.5;
        LeftFricwhemotorConfigs.Slot0.kV = 0.0;
        LeftFricwhemotorConfigs.Slot0.kA = 0;
        LeftFricwhemotorConfigs.Slot0.kP = 7;
        LeftFricwhemotorConfigs.Slot0.kI = 0;
        LeftFricwhemotorConfigs.Slot0.kD = 0.1;
        LeftFricwhemotorConfigs.MotionMagic.MotionMagicAcceleration = 100; 
        LeftFricwhemotorConfigs.MotionMagic.MotionMagicCruiseVelocity = 200; 
        LeftFricwhemotorConfigs.MotionMagic.MotionMagicExpo_kV = 0.12; 
        LeftFricwhemotorConfigs.MotionMagic.MotionMagicExpo_kA = 0.1; 
        LeftFricwhemotorConfigs.MotionMagic.MotionMagicJerk = 0; 
        LeftFrictionwheelMotor.getConfigurator().apply(LeftFricwhemotorConfigs);

        var MiddleFricwhemotorConfigs = new TalonFXConfiguration();
        MiddleFricwhemotorConfigs.Slot0.kS = 1.5;
        MiddleFricwhemotorConfigs.Slot0.kV = 0.0;
        MiddleFricwhemotorConfigs.Slot0.kA = 0;
        MiddleFricwhemotorConfigs.Slot0.kP = 7;
        MiddleFricwhemotorConfigs.Slot0.kI = 0;
        MiddleFricwhemotorConfigs.Slot0.kD = 0.1;
        MiddleFricwhemotorConfigs.MotionMagic.MotionMagicAcceleration = 100; 
        MiddleFricwhemotorConfigs.MotionMagic.MotionMagicCruiseVelocity = 200; 
        MiddleFricwhemotorConfigs.MotionMagic.MotionMagicExpo_kV = 0.12; 
        MiddleFricwhemotorConfigs.MotionMagic.MotionMagicExpo_kA = 0.1; 
        MiddleFricwhemotorConfigs.MotionMagic.MotionMagicJerk = 0; 
        MiddleFrictionwheelMotor.getConfigurator().apply(MiddleFricwhemotorConfigs);

        var RightFricwhemotorConfigs = new TalonFXConfiguration();
        RightFricwhemotorConfigs.Slot0.kS = 1.5;
        RightFricwhemotorConfigs.Slot0.kV = 0.0;
        RightFricwhemotorConfigs.Slot0.kA = 0;
        RightFricwhemotorConfigs.Slot0.kP = 7;
        RightFricwhemotorConfigs.Slot0.kI = 0;
        RightFricwhemotorConfigs.Slot0.kD = 0.1;
        RightFricwhemotorConfigs.MotionMagic.MotionMagicAcceleration = 100; 
        RightFricwhemotorConfigs.MotionMagic.MotionMagicCruiseVelocity = 200; 
        RightFricwhemotorConfigs.MotionMagic.MotionMagicExpo_kV = 0.12; 
        RightFricwhemotorConfigs.MotionMagic.MotionMagicExpo_kA = 0.1; 
        RightFricwhemotorConfigs.MotionMagic.MotionMagicJerk = 0; 
        RightFrictionwheelMotor.getConfigurator().apply(RightFricwhemotorConfigs);

        var IntakeballmotorConfigs = new TalonFXConfiguration();
        IntakeballmotorConfigs.Slot0.kS = 1.5;
        IntakeballmotorConfigs.Slot0.kV = 0.0;
        IntakeballmotorConfigs.Slot0.kA = 0;
        IntakeballmotorConfigs.Slot0.kP = 7;
        IntakeballmotorConfigs.Slot0.kI = 0;
        IntakeballmotorConfigs.Slot0.kD = 0.1;
        IntakeballmotorConfigs.MotionMagic.MotionMagicAcceleration = 100;
        IntakeballmotorConfigs.MotionMagic.MotionMagicCruiseVelocity = 200; 
        IntakeballmotorConfigs.MotionMagic.MotionMagicExpo_kV = 0.12; 
        IntakeballmotorConfigs.MotionMagic.MotionMagicExpo_kA = 0.1; 
        IntakeballmotorConfigs.MotionMagic.MotionMagicJerk = 0; 
        IntakeBallMotor.getConfigurator().apply(IntakeballmotorConfigs);
    }

    public void setIntakeShooterVelocity(double Velocity) { 
        LeftFrictionwheelMotor.setControl(AllFrictionwheelMotor_Request.withVelocity(Velocity));
        MiddleFrictionwheelMotor.setControl(AllFrictionwheelMotor_Request.withVelocity(Velocity));
        RightFrictionwheelMotor.setControl(AllFrictionwheelMotor_Request.withVelocity(Velocity));
    }


    public void setIntakeBallVelocity(double Velocity) {
        IntakeBallMotor.setControl(IntakeBallMotor_Request.withVelocity(Velocity));
    }

    public Command Frictionwheel_presstime(){
        return runOnce(
            () -> {Frictionwhepressed_times += 1;
            }
        );
    }

    public Command ShooterCommand() { 
        return runOnce(
            () -> { 
                if (Frictionwhepressed_times%2==0){
                    setIntakeShooterVelocity(Frictionwheelshootspeed);
                }
                else{
                    setIntakeShooterVelocity(Frictionwheelstopspeed);
                }
            }
        );
    }

    public Command Intakeball_presstime(){
        return runOnce(
            () -> {Intakeballpressed_times += 1;
            }
        );
    }

    public Command IntakeballCommand() { 
        return runOnce(
            () -> { 
                if (Intakeballpressed_times%2==0){
                    setIntakeBallVelocity(Intakeballspeed);
                }
                else{
                    setIntakeBallVelocity(Outtakeballstopspeed);
                }
            }
        );
    }

    public Command OuttakeEverything(){
        return runOnce(
            ()-> {
                setIntakeShooterVelocity(Frictionwheelretreatspeed);
                setIntakeBallVelocity(IntakeBallRetreatSpeed);
            }
        );
    }
}