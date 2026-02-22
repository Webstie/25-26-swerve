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
        Transport transport,
        double frictionWheelLaunchSpeed,
        double launch_angle
    ) {
        // 定义预热时间
        double warmupTime = Constants.LauncherConfig.WarmupSecond;

        // --- 1. Launcher 的逻辑流 (预热 -> 发射) ---
        // 注意：摩擦轮在两个阶段都要转，Feeder 只在第二阶段转
        Command launcherStream = Commands.sequence(
            // 第一阶段：预热 (摩擦轮转，Feeder停,调整角度)
        Commands.parallel(
            Commands.run(
                    () -> {
                            launcher.setFrictionWheelVelocity(frictionWheelLaunchSpeed);
                            launcher.setFeederVelocity(0);
                        }
                    )
                    .alongWith(launcher.AdjustAngleToPositionCommand(launch_angle))   
        )

            .withTimeout(warmupTime), // 运行指定时间后自动进入下一阶段

            // 第二阶段：发射 (摩擦轮转，Feeder转)
            Commands.run(
                () -> {
                    launcher.setFrictionWheelVelocity(frictionWheelLaunchSpeed);
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
                }, transport
            )
        );

        // --- 3. Intake 的逻辑流 (等待 -> 循环摆动) ---
        Command intakeStream = Commands.sequence(
            Commands.waitSeconds(warmupTime), // 等待预热
            // 这里直接将 Command 对象放入 sequence，而不是在 lambda 中创建
            Commands.parallel(intake.IntakeSwingSingleCommand().repeatedly()
                        .alongWith(Commands.run(() -> {
                            intake.setIntakeMotorVelocity(Constants.IntakeConfig.IntakeVelocity);
                            intake.setSupportMotorVelocity(Constants.IntakeConfig.SupportVelocity);
                        }))
                        )       
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
            System.out.println("finallydo xxxxxxxxxxxxxxxxxxxxxxxxxx");
            //摩擦轮停
            launcher.setFrictionWheelVelocity(0);
            //feeder轮停xxx
            launcher.setFeederVelocity(0);
            //Transport停
            transport.setTransportVelocity(0);
            //Intake停
            intake.setIntakeMotorVelocity(0);
            //搅拌停
            intake.setSupportMotorVelocity(0);
            //释放intakepitch
            intake.applyIntakePitchMotorNeutral();

        });
    }


    //自动发射（不等待预热，直接执行），路径移动时已经提前预热
    public static Command createAutoShootingCommand(
        Intake intake,
        Launcher launcher,
        Transport transport,
        double frictionWheelLaunchSpeed
        
    ) {

        // --- 1. Launcher 的逻辑流 (预热 -> 发射) ---
        // 注意：摩擦轮在两个阶段都要转，Feeder 只在第二阶段转
        Command launcherStream = Commands.sequence(
            // 第一阶段： (摩擦轮转，Feeder转,调整角度)
            Commands.parallel(
                Commands.run(
                    () -> {
                        launcher.setFrictionWheelVelocity(frictionWheelLaunchSpeed);
                        launcher.setFeederVelocity(Constants.LauncherConfig.FeederSpeed);
                    }
                )

            )

        );

        // --- 2. Transport 的逻辑流 (等待 -> 运行) ---
        Command transportStream = Commands.sequence(
            Commands.run(
                () -> 
                {
                    transport.setTransportVelocity(Constants.TransportConfig.TransportSpeed);
                    //在启动传送带的同时启动Support电机
                }, transport
            )
        );

        // --- 3. Intake 的逻辑流 (等待 -> 循环摆动) ---
        Command intakeStream = Commands.sequence(
            // 这里直接将 Command 对象放入 sequence，而不是在 lambda 中创建
        Commands.parallel(intake.IntakeSwingSingleCommand().repeatedly()
                        .alongWith(Commands.run(() -> {
                            intake.setIntakeMotorVelocity(Constants.IntakeConfig.IntakeVelocity);
                            intake.setSupportMotorVelocity(Constants.IntakeConfig.SupportVelocity);
                        }))
                        )    
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
            //摩擦轮停
            launcher.setFrictionWheelVelocity(0);
            //feeder轮停
            launcher.setFeederVelocity(0);
            //Transport停
            transport.setTransportVelocity(0);
            //Intake停
            intake.setIntakeMotorVelocity(0);
            //搅拌停
            intake.setSupportMotorVelocity(0);
            //释放intakepitch
            intake.applyIntakePitchMotorNeutral();

        });
    }
}


