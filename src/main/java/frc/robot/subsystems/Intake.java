package frc.robot.subsystems;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.NeutralOut;
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

    private int Intake_press_times = 0;
    private boolean IntakepitchPositionFlag = true;

    private final NeutralOut Neutral_Request = new NeutralOut(); // Coast/neutral release for intake pitch motor

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
        IntakePitchMotorConfigs.MotionMagic.MotionMagicAcceleration = 200; 
        IntakePitchMotorConfigs.MotionMagic.MotionMagicCruiseVelocity = 400; 
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
     * Releases the pitch motor to coast (neutral output).
     */
    public void applyIntakePitchMotorNeutral() {
        Intake_pitch_motor.setControl(Neutral_Request);
    }

    /** Returns true if the intake motor is currently running (odd press count). */
    public boolean isIntakeRunning() {
        return Intake_press_times % 2 == 1;
    }

    /** Returns the current intake pitch flag (true = up position). */
    public boolean getIntakePitchFlag() {
        return IntakepitchPositionFlag;
    }

    /** Stops intake motor and resets counter, keeping toggle logic in sync with actual motor state. */
    public void resetIntakeCounter() {
        Intake_press_times = 0;
    }

    /**
     * Called on teleop init: resets counter and pitch flag.
     * Ensures the first teleop button press works regardless of how auto ended.
     * Pitch flag is reset to true (assumes stowed); first Driver.x() press will deploy.
     */
    public void resetTeleopState() {
        Intake_press_times = 0;
        IntakepitchPositionFlag = true;
        setIntakeMotorVelocity(0);
        setSupportMotorVelocity(0);
    }

    /**
     * Sets intake roller velocity.
     */
    public void setIntakeMotorVelocity(double velocity) {
        Intake_motor.setControl(Intake_motor_Velocity_Request.withVelocity(velocity));
    }

    /**
     * Sets intake pitch motor position.
     */
    public void setPitchMotorPosition(double position) {
        Intake_pitch_motor.setControl(Intake_pitch_motor_Voltage_Request.withPosition(position));
    }

    /**
     * Returns current intake pitch motor position.
     */
    public double get_PitchMotorPosition() {
        return Intake_pitch_motor.getPosition().getValueAsDouble();
    }

    /**
     * Sets intake support roller velocity.
     */
    public void setSupportMotorVelocity(double velocity) {
        Intake_support_motor.setControl(Intake_support_motor_Velocity_Request.withVelocity(velocity));
    }

    /**
     * Toggles intake roller on/off (odd press count = on, even = off).
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
     * Outtakes until interrupted.
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
     * Toggles the intake pitch position flag.
     */
    public Command ChangePitchPositionSingleCommand() {
        return runOnce(
            () -> { 
                IntakepitchPositionFlag = !IntakepitchPositionFlag; 
            }
            
        );
    };

    /**
     * Increments the intake press counter to toggle speed.
     */
    public Command ChangeIntakeSpeedSingleCommand() {
        return runOnce(
            () -> { 
                Intake_press_times += 1;
            }
            
        );
    };

    /**
     * Sets intake to running state (press count = 1) for auto.
     */
    public Command SetIntakeSpeedOneSingleCommand() {
        return runOnce(
            () -> { 
                Intake_press_times = 1;
            }
            
        );
    };

    /**
     * Sets intake to stopped state (press count = 0) for auto.
     */
    public Command SetIntakeSpeedZeroSingleCommand() {
        return runOnce(
            () -> { 
                Intake_press_times = 0;
            }
            
        );
    };


    /**
     * Moves intake to the expected pitch position; holds until within tolerance.
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
        // .finallyDo(
        //     ()->{if (expected_position == Constants.IntakeConfig.IntakeDownPosition){
        //             applyIntakePitchMotorNeutral();
        //             }
        //         });
    }

    /**
     * Moves intake to expected pitch while simultaneously outtaking.
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
     * Outtakes for a fixed duration.
     */
    private Command OuttakeForSingleCommand(double seconds) {
        return startEnd(
            () -> setIntakeMotorVelocity(OuttakeVelocity),
            () -> setIntakeMotorVelocity(0)
        ).withTimeout(seconds);
    }
    
    /**
     * Swings intake between SwingUp and SwingDown positions.
     */
    public Command IntakeSwingSingleCommand() {
        return AdjustIntakePositionSingleCommand(IntakeSwingUpPosition)
            .andThen(new WaitCommand(SwingWaitTime))
            .andThen(AdjustIntakePositionSingleCommand(IntakeSwingDownPosition))
            .andThen(new WaitCommand(SwingWaitTime));
    }

    /**
     * Swings intake between SwingUp and Down positions for feeding.
     */
    public Command IntakeFeedingSwingSingleCommand() {
        return AdjustIntakePositionSingleCommand(IntakeSwingUpPosition)
            .andThen(new WaitCommand(SwingWaitTime))
            .andThen(AdjustIntakePositionSingleCommand(IntakeDownPosition))
            .andThen(new WaitCommand(SwingWaitTime));
    }

    /**
     * Swings intake while outtaking for clearing jams.
     */
    public Command OuttakeSwingSingleCommand() {
        return AdjustIntakePosition_WithOuttakeSingleCommand(IntakeSwingUpPosition)//up
            .andThen(OuttakeForSingleCommand(SwingWaitTime))
            .andThen(AdjustIntakePosition_WithOuttakeSingleCommand(IntakeSwingDownPosition))//down
            .andThen(OuttakeForSingleCommand(SwingWaitTime));
    }
}
