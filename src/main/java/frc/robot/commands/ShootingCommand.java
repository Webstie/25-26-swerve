package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ShooterSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.TransportSubsystem;

public class ShootingCommand extends Command{
    public static IntakeSubsystem intake;
    public static ShooterSubsystem shooter;
    public static TransportSubsystem transport;
    public ShootingCommand(IntakeSubsystem intakeSubsystem, ShooterSubsystem shooterSubsystem, TransportSubsystem transportSubsystem){
        intake = intakeSubsystem;
        shooter = shooterSubsystem;
        transport = transportSubsystem;
    }
    // @Override
    // public void execute(){}
    public static final Command Shoot(){
        return new ParallelCommandGroup(
            intake.swing_IntakePosition(),
            shooter.ShooterCommand(),
            transport.TransportIntakeCommand()
        );
    }
}
