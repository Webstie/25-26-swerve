package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Launcher;
import frc.robot.subsystems.Transport;

public class OuttakeCommand extends ParallelCommandGroup {
    public OuttakeCommand(
        Intake intake,
        Launcher launcher,
        Transport transport
    ) {
        addCommands(
            intake.OuttakeSwingSingleCommand().repeatedly(),
            launcher.OuttakeSingleCommand(),
            transport.TransportOuttakeSingleCommand()
        );
    }
}
