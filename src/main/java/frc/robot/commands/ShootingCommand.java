package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.TransportSubsystem;

public class ShootingCommand extends ParallelCommandGroup {
    public ShootingCommand(
        IntakeSubsystem intakeSubsystem,
        ShooterSubsystem shooterSubsystem,
        TransportSubsystem transportSubsystem
    ) {
        addCommands(
            // intakeSubsystem.swing_IntakePosition().repeatedly(),
            shooterSubsystem.ShooterCommand(),
            transportSubsystem.TransportIntakeCommand()
        );
    }
}
