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
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import static frc.robot.Constants.LauncherConfig.*;
import static frc.robot.Constants.TransportConfig.*;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;


public class Launcher extends SubsystemBase {

    private static final double ANGLE_TOLERANCE = 0.001;
    private static final double VELOCITY_TOLERANCE = 2.0; // rps

    private double frictionWheelVelocityTarget = 0.0;
    private double targetDistanceMeters = 0.0;
    private double fireBoostRps = 0.0;
    private double fireBoostEndTime = 0.0;
    private boolean intakeBrake = false;
    private final SlewRateLimiter velocityLimiter = new SlewRateLimiter(FrictionWheelVelocityRampRate);
    private final NeutralOut neutralRequest = new NeutralOut();

    private final TalonFX feederMotor = new TalonFX(FEEDER_MOTOR_ID, new CANBus("canivore"));
    private final TalonFX leftFrictionWheelMotor = new TalonFX(LEFT_FRICTIONWHEEL_MOTOR_ID, new CANBus("canivore"));
    private final TalonFX middleFrictionWheelMotor = new TalonFX(MIDDLE_FRICTIONWHEEL_MOTOR_ID, new CANBus("canivore"));
    private final TalonFX rightFrictionWheelMotor = new TalonFX(RIGHT_FRICTIONWHEEL_MOTOR_ID, new CANBus("canivore"));
    // Transport (conveyor) motor — on Rio CAN, moves balls from intake to shooter
    private final TalonFX transportMotor = new TalonFX(TRANSPORT_MOTOR_ID, new CANBus("rio"));
    private final SparkMax angleAdjustment;
    private final CANcoder angleEncoder;

    private final VelocityTorqueCurrentFOC frictionWheelRequest = new VelocityTorqueCurrentFOC(0.0).withSlot(0);
    private final VelocityTorqueCurrentFOC feederRequest = new VelocityTorqueCurrentFOC(0.0).withSlot(0);
    private final VelocityTorqueCurrentFOC transportRequest = new VelocityTorqueCurrentFOC(0).withSlot(0);

    public Launcher() {
        angleAdjustment = new SparkMax(SHOOTER_SPARKMAX_ID, MotorType.kBrushed);
        angleEncoder = new CANcoder(ANGLE_CANCODER_ID, "canivore");

        leftFrictionWheelMotor.getConfigurator().apply(createFrictionWheelMotorConfig());
        middleFrictionWheelMotor.getConfigurator().apply(createFrictionWheelMotorConfig());
        rightFrictionWheelMotor.getConfigurator().apply(createFrictionWheelMotorConfig());

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
        feederMotor.getConfigurator().apply(feederCfg);

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
        transportMotor.getConfigurator().apply(transportCfg);
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

    public void setIntakeBrake(boolean brake) {
        intakeBrake = brake;
    }

    public void setTargetDistance(double distanceMeters) {
        targetDistanceMeters = distanceMeters;
    }

    /** Boosts friction wheels by the current target distance (rps) for 2 seconds. */
    public void startFireBoost() {
        fireBoostRps = targetDistanceMeters;
        fireBoostEndTime = Timer.getFPGATimestamp() + 0.5;
    }

    @Override
    public void periodic() {
        if (frictionWheelVelocityTarget != 0.0) {
            double limitedVelocity = velocityLimiter.calculate(frictionWheelVelocityTarget);
            double boost = Timer.getFPGATimestamp() < fireBoostEndTime ? fireBoostRps : 0.0;
            applyFrictionWheelVelocity(limitedVelocity + boost);
        } else if (intakeBrake) {
            applyFrictionWheelVelocity(0);
        } else {
            applyFrictionWheelNeutral();
        }
    }

    private void applyFrictionWheelVelocity(double velocity) {
        leftFrictionWheelMotor.setControl(frictionWheelRequest.withVelocity(velocity));
        middleFrictionWheelMotor.setControl(frictionWheelRequest.withVelocity(velocity));
        // Right wheel is physically mounted mirrored — negate for consistent spin direction
        rightFrictionWheelMotor.setControl(frictionWheelRequest.withVelocity(-velocity));
    }

    public void applyFrictionWheelNeutral() {
        leftFrictionWheelMotor.setControl(neutralRequest);
        middleFrictionWheelMotor.setControl(neutralRequest);
        rightFrictionWheelMotor.setControl(neutralRequest);
    }

    public void setAngleVoltage(double voltage) {
        angleAdjustment.setVoltage(voltage);
    }

    /** True when friction wheels have reached their current velocity target. */
    public boolean isFrictionWheelReady() {
        if (frictionWheelVelocityTarget == 0) return false;
        double actual = leftFrictionWheelMotor.getVelocity().getValueAsDouble();
        return Math.abs(actual - frictionWheelVelocityTarget) < VELOCITY_TOLERANCE;
    }

    /** True when the angle encoder is within tolerance of the target position. */
    public boolean isAngleAtPosition(double targetPosition) {
        return Math.abs(angleEncoder.getAbsolutePosition().getValueAsDouble() - targetPosition) <= ANGLE_TOLERANCE;
    }

    public void setAngleToTarget(double targetPosition) {
        final double kP = 200.0;
        final double maxVoltage = 12.0;
        final double kS = 5.0;

        double error = targetPosition - angleEncoder.getAbsolutePosition().getValueAsDouble();
        if (Math.abs(error) <= ANGLE_TOLERANCE) {
            setAngleVoltage(0);
            return;
        }
        double voltage = MathUtil.clamp(kP * error + Math.copySign(kS, error), -maxVoltage, maxVoltage);
        setAngleVoltage(voltage);
    }

    public Command adjustAngleCommand(double voltage) {
        return startEnd(
            () -> setAngleVoltage(voltage),
            () -> setAngleVoltage(0)
        );
    }

    public void setFrictionWheelVelocity(double velocity) {
        frictionWheelVelocityTarget = velocity;
    }

    public void setFeederVelocity(double velocity) {
        feederMotor.setControl(feederRequest.withVelocity(velocity));
    }

    // Motor is physically mounted in reverse, so negate the requested velocity
    public void setTransportVelocity(double velocity) {
        transportMotor.setControl(transportRequest.withVelocity(-velocity));
    }

    public double getFeederVelocity() {
        return feederMotor.getVelocity().getValueAsDouble();
    }

    public Command launchCommand(double frictionWheelLaunchSpeed) {
        return startEnd(
            () -> {
                setFrictionWheelVelocity(frictionWheelLaunchSpeed);
                setFeederVelocity(FeederSpeed);
            },
            () -> {
                setFrictionWheelVelocity(0);
                setFeederVelocity(0);
            }
        );
    }

    public Command outtakeCommand() {
        return startEnd(
            () -> {
                setFrictionWheelVelocity(FrictionWheelOuttakeVelocity);
                setFeederVelocity(OuttakeBallspeed);
                setTransportVelocity(-TransportSpeed);
            },
            () -> {
                setFrictionWheelVelocity(0.0);
                setFeederVelocity(0.0);
                setTransportVelocity(0.0);
            }
        );
    }

    public Command transportIntakeCommand() {
        return startEnd(
            () -> setTransportVelocity(TransportSpeed),
            () -> setTransportVelocity(0)
        );
    }

    public Command transportOuttakeCommand() {
        return startEnd(
            () -> setTransportVelocity(-TransportSpeed),
            () -> setTransportVelocity(0)
        );
    }

    public Command shooterWarmupCommand(double frictionWheelLaunchSpeed) {
        return runOnce(() -> setFrictionWheelVelocity(frictionWheelLaunchSpeed));
    }
}
