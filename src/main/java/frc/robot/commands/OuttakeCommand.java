package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Launcher;
import frc.robot.subsystems.Transport;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CANdleSystem;
import frc.robot.Constants;


public class OuttakeCommand { // 注意：这里不再 extends ParallelCommandGroup
        // 创建一个静态方法来生成命令
        public static Command create(Intake intake, Launcher launcher, Transport transport, CANdleSystem candle) {
            return new ParallelCommandGroup(
                intake.OuttakeSwingSingleCommand().repeatedly(),
                launcher.OuttakeSingleCommand(),
                transport.TransportOuttakeSingleCommand()
            )
            // 关键点：使用 finallyDo 在命令结束或中断时执行逻辑
            .finallyDo(() -> {
                intake.applyIntakePitchMotorNeutral();
                candle.Changecolor(Constants.RobotState.State.Idle);
        });
    }
}