package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.TransportSubsystem;
import frc.robot.Constants.Shooter;

public class ShootingCommand extends SequentialCommandGroup {
    public ShootingCommand(
        IntakeSubsystem intakeSubsystem,
        ShooterSubsystem shooterSubsystem,
        TransportSubsystem transportSubsystem
    ) {
        addCommands(
            // intakeSubsystem.swing_IntakePosition().repeatedly(),
            shooterSubsystem.ShooterWarmupCommand().withTimeout(Shooter.WarmupSecond),
            shooterSubsystem.ShooterCommand(),
            transportSubsystem.TransportIntakeCommand()
        );
    }
}
