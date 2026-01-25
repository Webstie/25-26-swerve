package frc.robot.subsystems;

import javax.print.attribute.standard.PrintQuality;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;

import static frc.robot.Constants.Intake.*;

import java.util.function.BooleanSupplier;




public class IntakeSubsystem extends SubsystemBase {

    //声明请求
    private final TalonFX Intake_motor = new TalonFX(9,"rio");
    private final TalonFX Intake_pitch_motor = new TalonFX(10,"rio");

    private final CANcoder intakePitchEncoder = new CANcoder(5, "canivore");

    //声明控制请求
    private final VelocityTorqueCurrentFOC Intake_motor_Velocity_Request = new VelocityTorqueCurrentFOC(0.0).withSlot(0);

    private final MotionMagicVoltage Intake_pitch_motor_Voltage_Request = new MotionMagicVoltage(0.0).withSlot(0);

    public int Intake_press_times = 0;
    public boolean IntakepitchPositionFlag = false;


    //方法1：构造函数
    public IntakeSubsystem() {

        var IntakePitchEncoderConfigs = new CANcoderConfiguration();

        IntakePitchEncoderConfigs.MagnetSensor.MagnetOffset=0.642977;
        IntakePitchEncoderConfigs.MagnetSensor.AbsoluteSensorDiscontinuityPoint=0.5;
        IntakePitchEncoderConfigs.MagnetSensor.SensorDirection=SensorDirectionValue.Clockwise_Positive;


        var IntakemotorConfigs = new TalonFXConfiguration();
        IntakemotorConfigs.Slot0.kS = 0.2;
        IntakemotorConfigs.Slot0.kV = 0.0;
        IntakemotorConfigs.Slot0.kA = 0;
        IntakemotorConfigs.Slot0.kP = 3;
        IntakemotorConfigs.Slot0.kI = 0;
        IntakemotorConfigs.Slot0.kD = 0;
        IntakemotorConfigs.MotionMagic.MotionMagicAcceleration = 100; // Acceleration is around 40 rps/s
        IntakemotorConfigs.MotionMagic.MotionMagicCruiseVelocity = 200; // Unlimited cruise velocity
        IntakemotorConfigs.MotionMagic.MotionMagicExpo_kV = 0.12; // kV is around 0.12 V/rps
        IntakemotorConfigs.MotionMagic.MotionMagicExpo_kA = 0.1; // Use a slower kA of 0.1 V/(rps/s)
        IntakemotorConfigs.MotionMagic.MotionMagicJerk = 0; // Jerk is around 0

        Intake_motor.getConfigurator().apply(IntakemotorConfigs);

        var IntakePitchmotorConfigs = new TalonFXConfiguration();
        IntakePitchmotorConfigs.Feedback.FeedbackRemoteSensorID = intakePitchEncoder.getDeviceID(); // CANcoder的设备ID（这里是1）
        IntakePitchmotorConfigs.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.FusedCANcoder; // 反馈源：Fused CANcoder
        IntakePitchmotorConfigs.Feedback.RotorToSensorRatio = 50; // 电机转子与传感器的传动比（根据实际机械结构调整）
        IntakePitchmotorConfigs.Slot0.kS = 0.2;
        IntakePitchmotorConfigs.Slot0.kV = 0.0;
        IntakePitchmotorConfigs.Slot0.kA = 0;
        IntakePitchmotorConfigs.Slot0.kP = 3;
        IntakePitchmotorConfigs.Slot0.kI = 0;
        IntakePitchmotorConfigs.Slot0.kD = 0;
        IntakePitchmotorConfigs.MotionMagic.MotionMagicAcceleration = 100; // Acceleration is around 40 rps/s
        IntakePitchmotorConfigs.MotionMagic.MotionMagicCruiseVelocity = 200; // Unlimited cruise velocity
        IntakePitchmotorConfigs.MotionMagic.MotionMagicExpo_kV = 0.12; // kV is around 0.12 V/rps
        IntakePitchmotorConfigs.MotionMagic.MotionMagicExpo_kA = 0.1; // Use a slower kA of 0.1 V/(rps/s)
        IntakePitchmotorConfigs.MotionMagic.MotionMagicJerk = 0; // Jerk is around 0

        Intake_pitch_motor.getConfigurator().apply(IntakePitchmotorConfigs);

        //暂时省略，用默认参数

    }

    //其他方法写在这里

   /**
     * Sets the motor position.
     * 
     * @param position
     */
    public void setIntakeMotorVelocity(double velocity) {
        //将上面的请求用到电机上
        Intake_motor.setControl(Intake_motor_Velocity_Request.withVelocity(velocity));
    }

    public void setPitchMotorPosition(double position) {
        Intake_pitch_motor.setControl(Intake_pitch_motor_Voltage_Request.withPosition(position));
    }

    public double get_PitchMotorPosition() {
        return Intake_pitch_motor.getPosition().getValueAsDouble();
    }

    //一个电机，速冻控制，一个是位置控制，电压控制



    //命令写在这里
    /**
     * Set the motor to a specific position.
     * 
     * @return
     */
    public Command IntakeCommand() {
        return runOnce(
            () -> {if(Intake_press_times % 2 == 0){
                setIntakeMotorVelocity(0);
            }
            else{
                 setIntakeMotorVelocity(10);
                }
            }
        );
    }

    public Command OuttakeCommand() {

        return startEnd(
            () -> { setIntakeMotorVelocity(-10);
                  },

            () -> {setIntakeMotorVelocity(0);
                  }
            );
    }

    public Command Intake_up_presstimes() {

        return runOnce(
            () -> { Intake_press_times += 1; }
        );
    };

    public Command changePitchPositionFlag() {

        return runOnce(
            () -> { IntakepitchPositionFlag = !IntakepitchPositionFlag; }
        );
    };


    public Command adjust_IntakePosition(double expected_position) { 

        return runEnd(
            () -> {
                   setPitchMotorPosition(expected_position);
                  },

            () -> {
                   setPitchMotorPosition(get_PitchMotorPosition());
                  }
        

    ).until( ()->Math.abs(get_PitchMotorPosition() - expected_position) < 0.5);

    }
    
    public Command swing_IntakePosition() {
        return runEnd(
            () -> { adjust_IntakePosition(SwingIntakePosition1);
                  },

            () -> { adjust_IntakePosition(SwingIntakePosition2);
                  }
            ).until(()-> Math.abs(get_PitchMotorPosition() - SwingIntakePosition1) < 1)
            .andThen(new WaitCommand(SwingWaitTime));
    }
}
