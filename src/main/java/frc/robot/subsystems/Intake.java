package frc.robot.subsystems;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;

import static frc.robot.Constants.IntakeConfig.*;




public class Intake extends SubsystemBase {

    private final TalonFX Intake_motor = new TalonFX(INTAKE_MOTOR_ID, new CANBus("canivore"));
    private final TalonFX Intake_pitch_motor = new TalonFX(INTAKE_PITCH_MOTOR_ID,new CANBus("rio"));
    private final TalonFX Intake_support_motor = new TalonFX(INTAKE_SUPPORT_MOTOR_ID,new CANBus("canivore"));

    private final VelocityTorqueCurrentFOC Intake_motor_Velocity_Request = new VelocityTorqueCurrentFOC(0.0).withSlot(0);
    private final MotionMagicVoltage Intake_pitch_motor_Voltage_Request = new MotionMagicVoltage(0.0).withSlot(0);
    private final VelocityTorqueCurrentFOC Intake_support_motor_Velocity_Request = new VelocityTorqueCurrentFOC(0.0).withSlot(0);

    public int Intake_press_times = 0;
    public boolean IntakepitchPositionFlag = true;

    public Intake() {

        var IntakePitchEncoderConfigs = new CANcoderConfiguration();

        IntakePitchEncoderConfigs.MagnetSensor.MagnetOffset = 0.642977;
        IntakePitchEncoderConfigs.MagnetSensor.AbsoluteSensorDiscontinuityPoint = 0.5;
        IntakePitchEncoderConfigs.MagnetSensor.SensorDirection=SensorDirectionValue.Clockwise_Positive;


        var IntakeMotorConfigs = new TalonFXConfiguration();
        IntakeMotorConfigs.Slot0.kS = 0.0;
        IntakeMotorConfigs.Slot0.kV = 0.0;
        IntakeMotorConfigs.Slot0.kA = 0;
        IntakeMotorConfigs.Slot0.kP = 5;
        IntakeMotorConfigs.Slot0.kI = 0;
        IntakeMotorConfigs.Slot0.kD = 0;
        IntakeMotorConfigs.MotionMagic.MotionMagicAcceleration = 100; 
        IntakeMotorConfigs.MotionMagic.MotionMagicCruiseVelocity = 200; 
        IntakeMotorConfigs.MotionMagic.MotionMagicExpo_kV = 0.12; 
        IntakeMotorConfigs.MotionMagic.MotionMagicExpo_kA = 0.1; 
        IntakeMotorConfigs.MotionMagic.MotionMagicJerk = 0; 
        Intake_motor.getConfigurator().apply(IntakeMotorConfigs);

        var IntakePitchMotorConfigs = new TalonFXConfiguration();
        IntakePitchMotorConfigs.Slot0.kS = 0.0;
        IntakePitchMotorConfigs.Slot0.kV = 0.0;
        IntakePitchMotorConfigs.Slot0.kA = 0;
        IntakePitchMotorConfigs.Slot0.kP = 5;
        IntakePitchMotorConfigs.Slot0.kI = 0;
        IntakePitchMotorConfigs.Slot0.kD = 0;
        IntakePitchMotorConfigs.MotionMagic.MotionMagicAcceleration = 100; 
        IntakePitchMotorConfigs.MotionMagic.MotionMagicCruiseVelocity = 200; 
        IntakePitchMotorConfigs.MotionMagic.MotionMagicExpo_kV = 0.12; 
        IntakePitchMotorConfigs.MotionMagic.MotionMagicExpo_kA = 0.1; 
        IntakePitchMotorConfigs.MotionMagic.MotionMagicJerk = 0;
        IntakePitchMotorConfigs.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        Intake_pitch_motor.getConfigurator().apply(IntakePitchMotorConfigs);

        var IntakeSupportMotorConfigs = new TalonFXConfiguration();
        IntakeSupportMotorConfigs.Slot0.kS = 0.0;
        IntakeSupportMotorConfigs.Slot0.kV = 0.0;
        IntakeSupportMotorConfigs.Slot0.kA = 0;
        IntakeSupportMotorConfigs.Slot0.kP = 5;
        IntakeSupportMotorConfigs.Slot0.kI = 0;
        IntakeSupportMotorConfigs.Slot0.kD = 0;
        IntakeSupportMotorConfigs.MotionMagic.MotionMagicAcceleration = 100; 
        IntakeSupportMotorConfigs.MotionMagic.MotionMagicCruiseVelocity = 200; 
        IntakeSupportMotorConfigs.MotionMagic.MotionMagicExpo_kV = 0.12; 
        IntakeSupportMotorConfigs.MotionMagic.MotionMagicExpo_kA = 0.1; 
        IntakeSupportMotorConfigs.MotionMagic.MotionMagicJerk = 0;
        IntakeSupportMotorConfigs.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        Intake_support_motor.getConfigurator().apply(IntakeSupportMotorConfigs);
    }

    /**
    Intake速度设置接口
     */
    public void setIntakeMotorVelocity(double velocity) {
        Intake_motor.setControl(Intake_motor_Velocity_Request.withVelocity(velocity));
    }

    /**
    Intake Pitch位置设置接口
     */
    public void setPitchMotorPosition(double position) {
        Intake_pitch_motor.setControl(Intake_pitch_motor_Voltage_Request.withPosition(position));
    }

    /**
    Intake Pitch位置获取接口
     */
    public double get_PitchMotorPosition() {
        return Intake_pitch_motor.getPosition().getValueAsDouble();
    }

    /**
    Intake Support设置接口
     */
    public void setSupportMotorVelocity(double velocity) {
        Intake_support_motor.setControl(Intake_support_motor_Velocity_Request.withVelocity(velocity));
    }

    /**
    Intake单独命令
     */
    public Command IntakeSingleCommand() {
        return runOnce(
            () -> {if(Intake_press_times % 2 == 0){
                setIntakeMotorVelocity(0);
            }
            else{
                 setIntakeMotorVelocity(IntakeVelocity);
                }
            }
        );
    }

    /**
    Outtake单独命令
     */
    public Command OuttakeSingleCommand() {
        return startEnd(
            () -> { setIntakeMotorVelocity(OuttakeVelocity);
                  },

            () -> {setIntakeMotorVelocity(0);
                  }
            );
    }

    /**
    切换Intake Pitch位置单独命令
     */
    public Command ChangePitchPositionSingleCommand() {
        return runOnce(
            () -> { 
                IntakepitchPositionFlag = !IntakepitchPositionFlag; 
            }
            
        );
    };

    /**
    切换Intake速度单独命令
     */
    public Command ChangeIntakeSpeedSingleCommand() {
        return runOnce(
            () -> { 
                Intake_press_times += 1;
            }
            
        );
    };

    /**
    调整Intake位置单独命令
     */
    public Command AdjustIntakePositionSingleCommand(double expected_position) { 
        return runEnd(
            () -> {
                   setPitchMotorPosition(expected_position);
                  },
            () -> {
                   setPitchMotorPosition(get_PitchMotorPosition());
                  }
        ).until( ()->Math.abs(get_PitchMotorPosition() - expected_position) < 0.5);
    }

    /**
    调整Intake位置并同时Outtake的单独命令
     */
    private Command AdjustIntakePosition_WithOuttakeSingleCommand(double expected_position) {
        return runEnd(
            () -> {
                setPitchMotorPosition(expected_position);
                setIntakeMotorVelocity(OuttakeVelocity);
            },
            () -> {
                setPitchMotorPosition(get_PitchMotorPosition());
                setIntakeMotorVelocity(0);
            }
        ).until(() -> Math.abs(get_PitchMotorPosition() - expected_position) < 0.5);
    }

    /**
    Outtake持续时间单独命令
     */
    private Command OuttakeForSingleCommand(double seconds) {
        return startEnd(
            () -> setIntakeMotorVelocity(OuttakeVelocity),
            () -> setIntakeMotorVelocity(0)
        ).withTimeout(seconds);
    }
    
    /**
    Intake摇摆单独命令
     */
    public Command IntakeSwingSingleCommand() {
        return AdjustIntakePositionSingleCommand(IntakeSwingUpPosition)
            .andThen(new WaitCommand(SwingWaitTime))
            .andThen(AdjustIntakePositionSingleCommand(IntakeSwingDownPosition))
            .andThen(new WaitCommand(SwingWaitTime));
    }

    /**
    Outtake摇摆单独命令
     */
    public Command OuttakeSwingSingleCommand() {
        return AdjustIntakePosition_WithOuttakeSingleCommand(IntakeSwingUpPosition)
            .andThen(OuttakeForSingleCommand(SwingWaitTime))
            .andThen(AdjustIntakePosition_WithOuttakeSingleCommand(IntakeSwingDownPosition))
            .andThen(OuttakeForSingleCommand(SwingWaitTime));
    }
}
