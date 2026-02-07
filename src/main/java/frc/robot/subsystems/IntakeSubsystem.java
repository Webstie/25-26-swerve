package frc.robot.subsystems;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;

import static frc.robot.Constants.Intake.*;




public class IntakeSubsystem extends SubsystemBase {

    private final TalonFX Intake_motor = new TalonFX(INTAKE_MOTOR_ID, new CANBus("rio"));
    private final TalonFX Intake_pitch_motor = new TalonFX(INTAKE_PITCH_MOTOR_ID,new CANBus("rio"));
    private final CANcoder intakePitchEncoder = new CANcoder(INTAKE_PITCH_ENCODER_ID, new CANBus("rio"));

    private final VelocityTorqueCurrentFOC Intake_motor_Velocity_Request = new VelocityTorqueCurrentFOC(0.0).withSlot(0);
    private final MotionMagicVoltage Intake_pitch_motor_Voltage_Request = new MotionMagicVoltage(0.0).withSlot(0);

    public int Intake_press_times = 0;
    public boolean IntakepitchPositionFlag = false;

    public IntakeSubsystem() {

        var IntakePitchEncoderConfigs = new CANcoderConfiguration();

        IntakePitchEncoderConfigs.MagnetSensor.MagnetOffset = 0.642977;
        IntakePitchEncoderConfigs.MagnetSensor.AbsoluteSensorDiscontinuityPoint = 0.5;
        IntakePitchEncoderConfigs.MagnetSensor.SensorDirection=SensorDirectionValue.Clockwise_Positive;


        var IntakemotorConfigs = new TalonFXConfiguration();
        IntakemotorConfigs.Slot0.kS = 0.0;
        IntakemotorConfigs.Slot0.kV = 0.0;
        IntakemotorConfigs.Slot0.kA = 0;
        IntakemotorConfigs.Slot0.kP = 5;
        IntakemotorConfigs.Slot0.kI = 0;
        IntakemotorConfigs.Slot0.kD = 0;
        IntakemotorConfigs.MotionMagic.MotionMagicAcceleration = 100; 
        IntakemotorConfigs.MotionMagic.MotionMagicCruiseVelocity = 200; 
        IntakemotorConfigs.MotionMagic.MotionMagicExpo_kV = 0.12; 
        IntakemotorConfigs.MotionMagic.MotionMagicExpo_kA = 0.1; 
        IntakemotorConfigs.MotionMagic.MotionMagicJerk = 0; 

        Intake_motor.getConfigurator().apply(IntakemotorConfigs);

        var IntakePitchmotorConfigs = new TalonFXConfiguration();
        IntakePitchmotorConfigs.Feedback.FeedbackRemoteSensorID = intakePitchEncoder.getDeviceID();
        IntakePitchmotorConfigs.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.FusedCANcoder; 
        IntakePitchmotorConfigs.Feedback.RotorToSensorRatio = 50; 
        IntakePitchmotorConfigs.Slot0.kS = 0.0;
        IntakePitchmotorConfigs.Slot0.kV = 0.0;
        IntakePitchmotorConfigs.Slot0.kA = 0;
        IntakePitchmotorConfigs.Slot0.kP = 5;
        IntakePitchmotorConfigs.Slot0.kI = 0;
        IntakePitchmotorConfigs.Slot0.kD = 0;
        IntakePitchmotorConfigs.MotionMagic.MotionMagicAcceleration = 100; 
        IntakePitchmotorConfigs.MotionMagic.MotionMagicCruiseVelocity = 200; 
        IntakePitchmotorConfigs.MotionMagic.MotionMagicExpo_kV = 0.12; 
        IntakePitchmotorConfigs.MotionMagic.MotionMagicExpo_kA = 0.1; 
        IntakePitchmotorConfigs.MotionMagic.MotionMagicJerk = 0;
        IntakePitchmotorConfigs.MotorOutput.NeutralMode = NeutralModeValue.Coast;

        Intake_pitch_motor.getConfigurator().apply(IntakePitchmotorConfigs);
    }


    public void setIntakeMotorVelocity(double velocity) {
        Intake_motor.setControl(Intake_motor_Velocity_Request.withVelocity(velocity));
    }

    public void setPitchMotorPosition(double position) {
        Intake_pitch_motor.setControl(Intake_pitch_motor_Voltage_Request.withPosition(position));
    }

    public double get_PitchMotorPosition() {
        return Intake_pitch_motor.getPosition().getValueAsDouble();
    }

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


    public Command changePositionFlag() {
        return runOnce(
            () -> { 
                IntakepitchPositionFlag = !IntakepitchPositionFlag; 
                Intake_press_times += 1;
            }
            
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
        return adjust_IntakePosition(IntakeDownPosition)
            .andThen(new WaitCommand(SwingWaitTime))
            .andThen(adjust_IntakePosition(IntakeUpPosition))
            .andThen(new WaitCommand(SwingWaitTime));
    }
}
