package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.MathUtil;
import static frc.robot.Constants.ClimberConfig.*;
import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;


public class Climber extends SubsystemBase {
    public final TalonFX climberMotor = new TalonFX(CLIMBER_MOTOR_ID, new CANBus("canivore"));

    private final MotionMagicVoltage climberRequest = new MotionMagicVoltage(0).withSlot(0);
    private final VoltageOut zeroVoltageRequest = new VoltageOut(0.0);
    private static final double POSITION_TOLERANCE = 1.0;


    public Climber() {
        var climberMotorConfigs = new TalonFXConfiguration();

        climberMotorConfigs.Slot0.kP = 10;
        climberMotorConfigs.Slot0.kI = 0.0;
        climberMotorConfigs.Slot0.kD = 0.0;
        climberMotorConfigs.Slot0.kS = 0.5;
        climberMotorConfigs.Slot0.kV = 0.5;
        climberMotorConfigs.Slot0.kA = 0.0;
        climberMotorConfigs.MotionMagic.MotionMagicAcceleration = 1000;
        climberMotorConfigs.MotionMagic.MotionMagicCruiseVelocity = 50;
        climberMotorConfigs.MotionMagic.MotionMagicExpo_kA = 0.12;
        climberMotorConfigs.MotionMagic.MotionMagicExpo_kV = 0.1;
        climberMotorConfigs.MotionMagic.MotionMagicJerk = 0;
        climberMotorConfigs.MotorOutput.NeutralMode = NeutralModeValue.Brake;

        climberMotor.getConfigurator().apply(climberMotorConfigs);
    }

    public void releaseClimber() {
        climberMotor.setControl(zeroVoltageRequest);
    }

    /**
     * Sets climber motor to target position.
     */
    public void setPosition(double position) {
        climberMotor.setControl(climberRequest.withPosition(position));
    }

    /**
     * Returns current climber position.
     */
    public double getCurrentPosition() {
        return climberMotor.getPosition().getValueAsDouble();
    }

    public boolean isAtPosition(double targetPosition) {
        return Math.abs(getCurrentPosition() - targetPosition) < POSITION_TOLERANCE;
    }

    /**
     * Runs climber to top position and holds until reached.
     */
    public Command climbingProcessCommand() {
        return runEnd(
                () -> setPosition(ClimberTopPosition),
                () -> setPosition(getCurrentPosition())
            ).until(() -> isAtPosition(ClimberTopPosition));
    }
}
