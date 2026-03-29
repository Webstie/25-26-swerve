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
import static frc.robot.Constants.TransportConfig.*;

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
    // Transport (conveyor) motor — on Rio CAN, moves balls from intake to shooter
    private final TalonFX TransportMotor = new TalonFX(TRANSPORT_MOTOR_ID, new CANBus("rio"));
    private final SparkMax angleAdjustment;
    private final CANcoder angleEncoder;

    private final VelocityTorqueCurrentFOC AllFrictionwheelMotor_Request = new VelocityTorqueCurrentFOC(0.0).withSlot(0);
    private final VelocityTorqueCurrentFOC FeederMotor_Request = new VelocityTorqueCurrentFOC(0.0).withSlot(0);
    private final VelocityTorqueCurrentFOC TransportMotor_Request = new VelocityTorqueCurrentFOC(0).withSlot(0);

    public Launcher() {
        angleAdjustment = new SparkMax(SHOOTER_SPARKMAX_ID, MotorType.kBrushed);
        angleEncoder = new CANcoder(ANGLE_CANCODER_ID, "canivore");

        LeftFrictionwheelMotor.getConfigurator().apply(createFrictionWheelMotorConfig());
        MiddleFrictionwheelMotor.getConfigurator().apply(createFrictionWheelMotorConfig());
        RightFrictionwheelMotor.getConfigurator().apply(createFrictionWheelMotorConfig());

        var feederCfg = new TalonFXConfiguration();
        feederCfg.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        feederCfg.Slot0.kS = 0.0;
        feederCfg.Slot0.kV = 0.0;
        feederCfg.Slot0.kA = 0;
        feederCfg.Slot0.kP = 5;
        feederCfg.Slot0.kI = 0;
        feederCfg.Slot0.kD = 0;
        feederCfg.MotionMagic.MotionMagicAcceleration = 100;
        feederCfg.MotionMagic.MotionMagicCruiseVelocity = 200;
        feederCfg.MotionMagic.MotionMagicExpo_kV = 0.12;
        feederCfg.MotionMagic.MotionMagicExpo_kA = 0.1;
        feederCfg.MotionMagic.MotionMagicJerk = 0;
        FeederMotor.getConfigurator().apply(feederCfg);

        var transportCfg = new TalonFXConfiguration();
        transportCfg.Slot0.kS = 0.0;
        transportCfg.Slot0.kV = 0.0;
        transportCfg.Slot0.kA = 0;
        transportCfg.Slot0.kP = 5;
        transportCfg.Slot0.kI = 0;
        transportCfg.Slot0.kD = 0;
        transportCfg.MotionMagic.MotionMagicAcceleration = 100;
        transportCfg.MotionMagic.MotionMagicCruiseVelocity = 200;
        transportCfg.MotionMagic.MotionMagicExpo_kV = 0.12;
        transportCfg.MotionMagic.MotionMagicExpo_kA = 0.1;
        transportCfg.MotionMagic.MotionMagicJerk = 0;
        TransportMotor.getConfigurator().apply(transportCfg);
    }

    private static TalonFXConfiguration createFrictionWheelMotorConfig() {
        var cfg = new TalonFXConfiguration();
        cfg.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        cfg.Slot0.kS = 1.5;
        cfg.Slot0.kV = 0.15;
        cfg.Slot0.kA = 0;
        cfg.Slot0.kP = 6.5;
        cfg.Slot0.kI = 0;
        cfg.Slot0.kD = 0.2;
        cfg.MotionMagic.MotionMagicAcceleration = 100;
        cfg.MotionMagic.MotionMagicCruiseVelocity = 200;
        cfg.MotionMagic.MotionMagicExpo_kV = 0.12;
        cfg.MotionMagic.MotionMagicExpo_kA = 0.1;
        cfg.MotionMagic.MotionMagicJerk = 0;
        return cfg;
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

    private void applyFrictionWheelVelocity(double Velocity) {
        LeftFrictionwheelMotor.setControl(AllFrictionwheelMotor_Request.withVelocity(Velocity));
        MiddleFrictionwheelMotor.setControl(AllFrictionwheelMotor_Request.withVelocity(Velocity));
        // Right wheel is mounted mirrored — negate for consistent spin direction
        RightFrictionwheelMotor.setControl(AllFrictionwheelMotor_Request.withVelocity(-Velocity));
    }

    public void applyFrictionWheelNeutral() {
        LeftFrictionwheelMotor.setControl(Neutral_Request);
        MiddleFrictionwheelMotor.setControl(Neutral_Request);
        RightFrictionwheelMotor.setControl(Neutral_Request);
    }

    public void setAngleVoltage(double Voltage) {
        angleAdjustment.setVoltage(Voltage);
    }

    public void setAngleToTarget(double targetPosition) {
        double error = angleEncoder.getAbsolutePosition().getValueAsDouble() - targetPosition;
        if (Math.abs(error) <= ANGLE_TOLERANCE) {
            setAngleVoltage(0);
            return;
        }
        setAngleVoltage(error >= 0 ? -12 : 12);
    }

    public Command AdjustAngleToPositionCommand(double targetPosition) {
        final double kP = 200.0;
        final double maxVoltage = 12.0;
        final double kS = 5.0;

        return run(
            () -> {
                double current = angleEncoder.getAbsolutePosition().getValueAsDouble();
                double error = targetPosition - current;
                double absError = Math.abs(error);

                if (absError <= ANGLE_TOLERANCE) {
                    setAngleVoltage(0);
                    return;
                }

                double pOutput = kP * error;
                double feedforward = Math.copySign(kS, error);
                double voltage = MathUtil.clamp(pOutput + feedforward, -maxVoltage, maxVoltage);
                setAngleVoltage(voltage);
            }
        )
        .until(() -> Math.abs(targetPosition - angleEncoder.getAbsolutePosition().getValueAsDouble()) <= ANGLE_TOLERANCE)
        .withTimeout(1.5)
        .finallyDo(() -> setAngleVoltage(0));
    }

    public Command AdjustAngleSingleCommand(double Voltage) {
        return startEnd(
            () -> setAngleVoltage(Voltage),
            () -> setAngleVoltage(0)
        );
    }

    public void setFrictionWheelVelocity(double Velocity) {
        frictionWheelVelocityTarget = Velocity;
    }

    public void setFeederVelocity(double Velocity) {
        FeederMotor.setControl(FeederMotor_Request.withVelocity(Velocity));
    }

    // Motor is physically mounted in reverse, so negate the requested velocity
    public void setTransportVelocity(double velocity) {
        TransportMotor.setControl(TransportMotor_Request.withVelocity(-velocity));
    }

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

    public Command OuttakeSingleCommand() {
        return startEnd(
            () -> {
                setFrictionWheelVelocity(FrictionWheelOuttakeVelocity);
                setFeederVelocity(OuttakeBallspeed);
            },
            () -> {
                setFrictionWheelVelocity(0.0);
                setFeederVelocity(0.0);
            }
        );
    }

    public Command TransportIntakeSingleCommand() {
        return startEnd(
            () -> setTransportVelocity(TransportSpeed),
            () -> setTransportVelocity(0)
        );
    }

    public Command TransportOuttakeSingleCommand() {
        return startEnd(
            () -> setTransportVelocity(-TransportSpeed),
            () -> setTransportVelocity(0)
        );
    }

    public Command ShooterWarmupSingleCommand(double FrictionWheelLaunchSpeed) {
        return runOnce(() -> setFrictionWheelVelocity(FrictionWheelLaunchSpeed));
    }
}
