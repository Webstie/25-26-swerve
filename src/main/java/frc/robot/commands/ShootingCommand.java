package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Launcher;
import frc.robot.subsystems.Transport;

import frc.robot.Constants;

public class ShootingCommand extends SequentialCommandGroup {
    // public ShootingCommand(
    //     IntakeSubsystem intakeSubsystem,
    //     ShooterSubsystem shooterSubsystem,
    //     TransportSubsystem transportSubsystem
    // ) {
    //     addCommands(
    //         shooterSubsystem.ShooterWarmupCommand(),
            
    //         new WaitCommand(Shooter.WarmupSecond),

    //         new ParallelCommandGroup(
    //             //intakeSubsystem.swing_IntakePosition().repeatedly(),
    //             shooterSubsystem.ShooterCommand(),
    //             transportSubsystem.TransportIntakeCommand()
    //         )
    //     );
    // }

    /**
     * 创建一个射击命令，包含预热、摆动Intake和输料的逻辑,按下按钮时开始射击，松开按钮时停止射击
     * 
     * @param intakeSubsystem Intake子系统实例
     * @param shooterSubsystem Shooter子系统实例
     * @param transportSubsystem Transport子系统实例
     * @return 一个Command对象，表示射击命令
     */
    public static Command createShootingCommand(
        Intake intake,
        Launcher launcher,
        Transport transport
    ) {
        return Commands.startEnd(
            // 开始动作（按下按钮时执行）
            () -> {
                // 1. 启动发射器预热
                launcher.setFrictionWheelVelocity(Constants.LauncherConfig.FrictionWheelLaunchSpeed);
                
                // 2. 创建一个命令：等待预热时间后，同时启动摆动Intake和输料
                Commands.sequence(
                    // 等待预热时间
                    Commands.waitSeconds(Constants.LauncherConfig.WarmupSecond),
                    
                    Commands.runOnce(() -> {
                            transport.setTransportVelocity(Constants.TransportConfig.TransportSpeed);
                            launcher.setFeederVelocity(Constants.LauncherConfig.FeederSpeed);
                            //intake.IntakeSwingSingleCommand().repeatedly();
                    })

                ).schedule();
            },
            
            // 结束动作（松开按钮时执行）
            () -> {
                // 停止所有电机，缓慢停止
                launcher.setFrictionWheelVelocity(0);
                transport.setTransportVelocity(0);
                launcher.setFeederVelocity(0);
                
                // // 停止Intake摆动并回到下方位置
                // intake.setPitchMotorPosition(
                //     Constants.IntakeConfig.IntakeDownPosition
                // );
            }
        );
    }
}


