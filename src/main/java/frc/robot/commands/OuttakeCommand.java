package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Launcher;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CANdleSystem;


public class OuttakeCommand {
    public static Command create(Intake intake, Launcher launcher, CANdleSystem candle) {
        return new ParallelCommandGroup(
            intake.OuttakeSwingSingleCommand().repeatedly(),
            launcher.OuttakeSingleCommand(),
            launcher.TransportOuttakeSingleCommand()
        )
        .finallyDo(() -> {
            intake.applyIntakePitchMotorNeutral();
            intake.resetIntakeCounter();
            candle.restoreBackground();
        });
    }
}
