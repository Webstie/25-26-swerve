package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Launcher;
import frc.robot.subsystems.Transport;

import frc.robot.Constants;

public class ShootingCommand extends SequentialCommandGroup {
    /**
     * 创建一个射击命令
     * 逻辑：按下时：
     * 1. Launcher: 先预热摩擦轮 -> (预热结束后) 摩擦轮继续转 + 供弹轮(Feeder)转
     * 2. Transport: 等待预热时间 -> 开启输送带
     * 3. Intake: 等待预热时间 -> 循环执行摆动
     * 松开时：全部停止
     */
    public static Command createShootingCommand(
        Intake intake,
        Launcher launcher,
        Transport transport
    ) {
        // 定义预热时间
        double warmupTime = Constants.LauncherConfig.WarmupSecond;

        // --- 1. Launcher 的逻辑流 (预热 -> 发射) ---
        // 注意：摩擦轮在两个阶段都要转，Feeder 只在第二阶段转
        Command launcherStream = Commands.sequence(
            // 第一阶段：预热 (摩擦轮转，Feeder停)
            Commands.run(
                () -> {
                    launcher.setFrictionWheelVelocity(Constants.LauncherConfig.FrictionWheelLaunchSpeed);
                    launcher.setFeederVelocity(0);
                }, launcher
            ).withTimeout(warmupTime), // 运行指定时间后自动进入下一阶段

            // 第二阶段：发射 (摩擦轮转，Feeder转)
            Commands.run(
                () -> {
                    launcher.setFrictionWheelVelocity(Constants.LauncherConfig.FrictionWheelLaunchSpeed);
                    launcher.setFeederVelocity(Constants.LauncherConfig.FeederSpeed);
                }, launcher
            )
        );

        // --- 2. Transport 的逻辑流 (等待 -> 运行) ---
        Command transportStream = Commands.sequence(
            Commands.waitSeconds(warmupTime), // 等待预热
            Commands.run(
                () -> 
                {
                    transport.setTransportVelocity(Constants.TransportConfig.TransportSpeed);
                    //在启动传送带的同时启动Support电机
                    intake.setSupportMotorVelocity(Constants.IntakeConfig.SupportVelocity);
                }, transport
            )
        );

        // --- 3. Intake 的逻辑流 (等待 -> 循环摆动) ---
        Command intakeStream = Commands.sequence(
            Commands.waitSeconds(warmupTime), // 等待预热
            // 这里直接将 Command 对象放入 sequence，而不是在 lambda 中创建
            Commands.parallel(intake.IntakeSwingSingleCommand().repeatedly(),
                            Commands.run(()->intake.setSupportMotorVelocity(Constants.IntakeConfig.IntakeVelocity)))
           
        );

        // --- 组合所有流 ---
        // 使用 ParallelCommandGroup 同时运行这三条线
        return Commands.parallel(
            launcherStream,
            transportStream,
            intakeStream
        )
        // 关键：finallyDo 确保无论命令是正常结束还是被中断(松开按键)，都会执行清理
        .finallyDo((interrupted) -> {
            launcher.setFrictionWheelVelocity(0);
            launcher.setFeederVelocity(0);
            transport.setTransportVelocity(0);
            intake.setSupportMotorVelocity(0);
            // 停止 Intake 摆动并复位
            intake.setPitchMotorPosition(Constants.IntakeConfig.IntakeDownPosition);
        });
    }
}


