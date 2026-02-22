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
            //intakeSubsystem.swing_OuttakePosition().repeatedly(),
            launcher.OuttakeSingleCommand(),
            transport.TransportOuttakeSingleCommand()
        );
    }
}
