package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.TransportSubsystem;

public class OuttakeCommand extends ParallelCommandGroup {
    public OuttakeCommand(
        IntakeSubsystem intakeSubsystem,
        ShooterSubsystem shooterSubsystem,
        TransportSubsystem transportSubsystem
    ) {
        addCommands(
            //intakeSubsystem.swing_OuttakePosition().repeatedly(),
            shooterSubsystem.OuttakeSingleCommand(),
            transportSubsystem.TransportOuttakeSingleCommand()
        );
    }
}
