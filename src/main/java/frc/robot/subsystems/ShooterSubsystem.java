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
import static frc.robot.Constants.Shooter.*;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkMax;

public class ShooterSubsystem extends SubsystemBase {
    // private final SparkMax shooterAdjustment;
    private double shooterVelocityTarget = 0.0;
    private final SlewRateLimiter shooterVelocityLimiter =
        new SlewRateLimiter(FrictionwheelVelocityRampRate);
    private final NeutralOut Neutral_Request = new NeutralOut();

    private final TalonFX IntakeBallMotor = new TalonFX(FEEDER_MOTOR_ID, new CANBus("canivore"));
    private final TalonFX LeftFrictionwheelMotor = new TalonFX(LEFT_FRICTIONWHEEL_MOTOR_ID, new CANBus("canivore"));
    private final TalonFX MiddleFrictionwheelMotor = new TalonFX(MIDDLE_FRICTIONWHEEL_MOTOR_ID, new CANBus("canivore"));
    private final TalonFX RightFrictionwheelMotor = new TalonFX(RIGHT_FRICTIONWHEEL_MOTOR_ID, new CANBus("canivore"));

   private final VelocityTorqueCurrentFOC AllFrictionwheelMotor_Request = new VelocityTorqueCurrentFOC(0.0).withSlot(0);
   private final VelocityTorqueCurrentFOC IntakeBallMotor_Request = new VelocityTorqueCurrentFOC(0.0).withSlot(0);
  
    
    public ShooterSubsystem() {
        // shooterAdjustment = new SparkMax(SHOOTER_SPARKMAX_ID, MotorType.kBrushed);

        var LeftFricwhemotorConfigs = new TalonFXConfiguration();
        LeftFricwhemotorConfigs.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        LeftFricwhemotorConfigs.Slot0.kS = 0.0;
        LeftFricwhemotorConfigs.Slot0.kV = 0.0;
        LeftFricwhemotorConfigs.Slot0.kA = 0;
        LeftFricwhemotorConfigs.Slot0.kP = 5;
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
        MiddleFricwhemotorConfigs.Slot0.kS = 0.0;
        MiddleFricwhemotorConfigs.Slot0.kV = 0.0;
        MiddleFricwhemotorConfigs.Slot0.kA = 0;
        MiddleFricwhemotorConfigs.Slot0.kP = 5;
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
        RightFricwhemotorConfigs.Slot0.kS = 0.0;
        RightFricwhemotorConfigs.Slot0.kV = 0.0;
        RightFricwhemotorConfigs.Slot0.kA = 0;
        RightFricwhemotorConfigs.Slot0.kP = 5;
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
        if (shooterVelocityTarget == 0.0) {
            applyShooterNeutral();
        } else {
            double limitedVelocity = shooterVelocityLimiter.calculate(shooterVelocityTarget);
            applyShooterVelocity(limitedVelocity);
        }
    }

    private void applyShooterVelocity(double Velocity) { 
        LeftFrictionwheelMotor.setControl(AllFrictionwheelMotor_Request.withVelocity(Velocity));
        MiddleFrictionwheelMotor.setControl(AllFrictionwheelMotor_Request.withVelocity(Velocity));
        RightFrictionwheelMotor.setControl(AllFrictionwheelMotor_Request.withVelocity(-Velocity));
    }

    private void applyShooterNeutral() {
        LeftFrictionwheelMotor.setControl(Neutral_Request);
        MiddleFrictionwheelMotor.setControl(Neutral_Request);
        RightFrictionwheelMotor.setControl(Neutral_Request);
    }

    public void setShooterVelocity(double Velocity) { 
        shooterVelocityTarget = Velocity;
    }

    public void setIntakeVelocity(double Velocity) {
        IntakeBallMotor.setControl(IntakeBallMotor_Request.withVelocity(Velocity));
    }

    public Command ShooterCommand() { 
        return startEnd(
            () -> { 
                setShooterVelocity(Frictionwheelshootspeed);
                setIntakeVelocity(Intakeballspeed);
            },
            () -> {
                setShooterVelocity(0);
                setIntakeVelocity(0);
            }
        );
    }
    public Command ShooterOuttakeCommand(){
        return startEnd(
            ()->{
                setShooterVelocity(-40.0);
                setIntakeVelocity(OuttakeBallspeed);
            },
            ()->{
                setShooterVelocity(0.0);
                setIntakeVelocity(0.0);
            }
        );
    }
    public Command ShooterWarmupCommand() { 
        return runOnce(
            () -> { 
                setShooterVelocity(Frictionwheelshootspeed);
            }
        );
    }
}
