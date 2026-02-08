package frc.robot.subsystems;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.hardware.TalonFX;
import static frc.robot.Constants.Transport.*;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class TransportSubsystem extends SubsystemBase {

    private final TalonFX Transport_motor = new TalonFX(TRANSPORT_MOTOR_ID, new CANBus("canivore"));

    private final VelocityTorqueCurrentFOC Transport_motor_Velocity_Request = new VelocityTorqueCurrentFOC(0).withSlot(0);

    public int transport_press_times = 0;

    public TransportSubsystem() {

        var TransportmotorConfigs = new TalonFXConfiguration();
        TransportmotorConfigs.Slot0.kS = 0.0;
        TransportmotorConfigs.Slot0.kV = 0.0;
        TransportmotorConfigs.Slot0.kA = 0;
        TransportmotorConfigs.Slot0.kP = 5;
        TransportmotorConfigs.Slot0.kI = 0;
        TransportmotorConfigs.Slot0.kD = 0;
        TransportmotorConfigs.MotionMagic.MotionMagicAcceleration = 100; // Acceleration is around 40 rps/s
        TransportmotorConfigs.MotionMagic.MotionMagicCruiseVelocity = 200; // Unlimited cruise velocity
        TransportmotorConfigs.MotionMagic.MotionMagicExpo_kV = 0.12; // kV is around 0.12 V/rps
        TransportmotorConfigs.MotionMagic.MotionMagicExpo_kA = 0.1; // Use a slower kA of 0.1 V/(rps/s)
        TransportmotorConfigs.MotionMagic.MotionMagicJerk = 0; // Jerk is around 0

        Transport_motor.getConfigurator().apply(TransportmotorConfigs);
    }

    public void setTransportMotorVelocity(double velocity) {
        Transport_motor.setControl(Transport_motor_Velocity_Request.withVelocity(velocity));
    }

    
    public Command TransportIntakeCommand() {
        return startEnd(
            () -> { 
                setTransportMotorVelocity(-TransportSpeed);
                  },
                  
            () -> {
                setTransportMotorVelocity(0);
                  }
        );
    }

    public Command TransportOuttakeCommand() {
        return startEnd(
            () -> { 
                setTransportMotorVelocity(TransportSpeed*3);
                  },
                  
            () -> {
                setTransportMotorVelocity(0);
                  }
        );
    }
}   