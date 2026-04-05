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

    private final TalonFX intakeLeftMotor = new TalonFX(INTAKE_LEFT_MOTOR_ID, new CANBus("canivore"));
    private final TalonFX intakeRightMotor = new TalonFX(INTAKE_RIGHT_MOTOR_ID, new CANBus("canivore"));
    private final TalonFX intakePitchMotor = new TalonFX(INTAKE_PITCH_MOTOR_ID, new CANBus("rio"));

    private final VelocityTorqueCurrentFOC intakeLeftMotorRequest = new VelocityTorqueCurrentFOC(0.0).withSlot(0);
    private final VelocityTorqueCurrentFOC intakeRightMotorRequest = new VelocityTorqueCurrentFOC(0.0).withSlot(0);
    private final MotionMagicVoltage intakePitchMotorRequest = new MotionMagicVoltage(0.0).withSlot(0);

    private int intakePressCount = 0;
    private boolean intakePitchPositionFlag = true;

    private final NeutralOut neutralRequest = new NeutralOut(); // Coast/neutral release for intake pitch motor

    public Intake() {

        var intakePitchEncoderConfigs = new CANcoderConfiguration();

        intakePitchEncoderConfigs.MagnetSensor.MagnetOffset = 0.642977;
        intakePitchEncoderConfigs.MagnetSensor.AbsoluteSensorDiscontinuityPoint = 0.5;
        intakePitchEncoderConfigs.MagnetSensor.SensorDirection = SensorDirectionValue.Clockwise_Positive;


        var intakeMotorConfigs = new TalonFXConfiguration();
        intakeMotorConfigs.Slot0.kS = 0.0;
        intakeMotorConfigs.Slot0.kV = 0.0;
        intakeMotorConfigs.Slot0.kA = 0;
        intakeMotorConfigs.Slot0.kP = 5;
        intakeMotorConfigs.Slot0.kI = 0;
        intakeMotorConfigs.Slot0.kD = 0;
        intakeMotorConfigs.MotionMagic.MotionMagicAcceleration = 100;
        intakeMotorConfigs.MotionMagic.MotionMagicCruiseVelocity = 200;
        intakeMotorConfigs.MotionMagic.MotionMagicExpo_kV = 0.12;
        intakeMotorConfigs.MotionMagic.MotionMagicExpo_kA = 0.1;
        intakeMotorConfigs.MotionMagic.MotionMagicJerk = 0;
        intakeLeftMotor.getConfigurator().apply(intakeMotorConfigs);
        intakeRightMotor.getConfigurator().apply(intakeMotorConfigs);

        var intakePitchMotorConfigs = new TalonFXConfiguration();
        intakePitchMotorConfigs.Slot0.kS = 0.0;
        intakePitchMotorConfigs.Slot0.kV = 0.0;
        intakePitchMotorConfigs.Slot0.kA = 0;
        intakePitchMotorConfigs.Slot0.kP = 5;
        intakePitchMotorConfigs.Slot0.kI = 0;
        intakePitchMotorConfigs.Slot0.kD = 0;
        intakePitchMotorConfigs.MotionMagic.MotionMagicAcceleration = 200;
        intakePitchMotorConfigs.MotionMagic.MotionMagicCruiseVelocity = 400;
        intakePitchMotorConfigs.MotionMagic.MotionMagicExpo_kV = 0.12;
        intakePitchMotorConfigs.MotionMagic.MotionMagicExpo_kA = 0.1;
        intakePitchMotorConfigs.MotionMagic.MotionMagicJerk = 0;
        intakePitchMotorConfigs.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        intakePitchMotor.getConfigurator().apply(intakePitchMotorConfigs);
    }

    /**
     * Releases the pitch motor to coast (neutral output).
     */
    public void applyIntakePitchMotorNeutral() {
        intakePitchMotor.setControl(neutralRequest);
    }

    /** Returns true if the intake motor is currently running (odd press count). */
    public boolean isIntakeRunning() {
        return intakePressCount % 2 == 1;
    }

    /** Returns the current intake pitch flag (true = up position). */
    public boolean getIntakePitchFlag() {
        return intakePitchPositionFlag;
    }

    /** Stops intake motor and resets counter, keeping toggle logic in sync with actual motor state. */
    public void resetIntakeCounter() {
        intakePressCount = 0;
    }

    /**
     * Called on teleop init: resets counter and pitch flag.
     * Ensures the first teleop button press works regardless of how auto ended.
     * Pitch flag is reset to true (assumes stowed); first Driver.x() press will deploy.
     */
    public void resetTeleopState() {
        intakePressCount = 0;
        intakePitchPositionFlag = true;
        setIntakeMotorVelocity(0);
    }

    /**
     * Sets both intake roller motors (left and right) to the given velocity.
     */
    public void setIntakeMotorVelocity(double velocity) {
        intakeLeftMotor.setControl(intakeLeftMotorRequest.withVelocity(velocity));
        intakeRightMotor.setControl(intakeRightMotorRequest.withVelocity(-velocity));
    }

    /**
     * Sets intake pitch motor position.
     */
    public void setPitchMotorPosition(double position) {
        intakePitchMotor.setControl(intakePitchMotorRequest.withPosition(position));
    }

    /**
     * Returns current intake pitch motor position.
     */
    public double getPitchMotorPosition() {
        return intakePitchMotor.getPosition().getValueAsDouble();
    }

    /**
     * Toggles intake roller on/off (odd press count = on, even = off).
     */
    public Command intakeCommand() {
        return runOnce(
            () -> {
                if (intakePressCount % 2 == 0) {
                    setIntakeMotorVelocity(0);
                } else {
                    setIntakeMotorVelocity(IntakeVelocity);
                }
            }
        );
    }

    /**
     * Outtakes until interrupted.
     */
    public Command outtakeCommand() {
        return startEnd(
            () -> setIntakeMotorVelocity(OuttakeVelocity),
            () -> setIntakeMotorVelocity(0)
        );
    }

    /**
     * Toggles the intake pitch position flag.
     */
    public Command changePitchPositionCommand() {
        return runOnce(() -> intakePitchPositionFlag = !intakePitchPositionFlag);
    }

    /**
     * Increments the intake press counter to toggle speed.
     */
    public Command changeIntakeSpeedCommand() {
        return runOnce(() -> intakePressCount += 1);
    }

    /**
     * Sets intake to running state (press count = 1) for auto.
     */
    public Command setIntakeSpeedOneCommand() {
        return runOnce(() -> intakePressCount = 1);
    }

    /**
     * Sets intake to stopped state (press count = 0) for auto.
     */
    public Command setIntakeSpeedZeroCommand() {
        return runOnce(() -> intakePressCount = 0);
    }

    /**
     * Moves intake to the expected pitch position; holds until within tolerance.
     */
    public Command adjustIntakePositionCommand(double expectedPosition) {
        return runEnd(
            () -> setPitchMotorPosition(expectedPosition),
            () -> setPitchMotorPosition(getPitchMotorPosition())
        ).until(() -> Math.abs(getPitchMotorPosition() - expectedPosition) < 0.5);
    }

    /**
     * Moves intake to expected pitch while simultaneously outtaking.
     */
    private Command adjustIntakePositionWithOuttakeCommand(double expectedPosition) {
        return runEnd(
            () -> {
                setPitchMotorPosition(expectedPosition);
                setIntakeMotorVelocity(OuttakeVelocity);
            },
            () -> {
                setPitchMotorPosition(getPitchMotorPosition());
                setIntakeMotorVelocity(0);
            }
        ).until(() -> Math.abs(getPitchMotorPosition() - expectedPosition) < 0.5);
    }

    /**
     * Outtakes for a fixed duration.
     */
    private Command outtakeForCommand(double seconds) {
        return startEnd(
            () -> setIntakeMotorVelocity(OuttakeVelocity),
            () -> setIntakeMotorVelocity(0)
        ).withTimeout(seconds);
    }

    /**
     * Swings intake between SwingUp and SwingDown positions.
     */
    public Command intakeSwingCommand() {
        return adjustIntakePositionCommand(IntakeSwingUpPosition)
            .andThen(new WaitCommand(SwingWaitTime))
            .andThen(adjustIntakePositionCommand(IntakeSwingDownPosition))
            .andThen(new WaitCommand(SwingWaitTime));
    }

    /**
     * Swings intake between SwingUp and Down positions for feeding.
     */
    public Command intakeFeedingSwingCommand() {
        return adjustIntakePositionCommand(IntakeSwingUpPosition)
            .andThen(new WaitCommand(SwingWaitTime))
            .andThen(adjustIntakePositionCommand(IntakeDownPosition))
            .andThen(new WaitCommand(SwingWaitTime));
    }

    /**
     * Swings intake while outtaking for clearing jams.
     */
    public Command outtakeSwingCommand() {
        return adjustIntakePositionWithOuttakeCommand(IntakeSwingUpPosition)
            .andThen(outtakeForCommand(SwingWaitTime))
            .andThen(adjustIntakePositionWithOuttakeCommand(IntakeSwingDownPosition))
            .andThen(outtakeForCommand(SwingWaitTime));
    }
}
