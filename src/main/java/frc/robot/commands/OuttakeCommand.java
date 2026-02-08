package frc.robot.commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.TransportSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class OuttakeCommand extends ParallelCommandGroup {
    public OuttakeCommand(
        IntakeSubsystem intakeSubsystem,
        TransportSubsystem transportSubsystem,
        ShooterSubsystem shooterSubsystem
    ) {
        addCommands(
            intakeSubsystem.OuttakeCommand(),
            shooterSubsystem.OuttakeCommand(),
            transportSubsystem.TransportOuttakeCommand()
        );
    }
}
