package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Launcher;
import frc.robot.subsystems.Transport;
import frc.robot.subsystems.CANdleSystem;
import frc.robot.subsystems.CommandSwerveDrivetrain;

import frc.robot.Constants;

public class ShootingCommand extends SequentialCommandGroup {
    private static final double PITCH_LEAD_RAD_PER_MPS = 0.025;
    private static final double MAX_PITCH_LEAD_RAD = 0.02;
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
        // 注意：摩擦轮在两个阶段都要转，Feeder 只$在第二阶段转
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
            ).withTimeout(warmupTime), // 运行指定时间后自动进入下一阶段

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
                () -> {
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
            launcher.setAngleVoltage(0);
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
                () -> {
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
            // intake.setIntakeMotorVelocity(0);
            //搅拌停
            intake.setSupportMotorVelocity(0);
            //释放intakepitch
            intake.applyIntakePitchMotorNeutral();

        });
    }

    public static Command createDynamicShootingCommand(
        CommandSwerveDrivetrain drive,
        Intake intake,
        Launcher launcher,
        Transport transport,
        Translation2d blueCenterPosition
    ) {
        double warmupSeconds = Constants.LauncherConfig.WarmupSecond;
        Timer warmupTimer = new Timer();

        Command launcherStream = Commands.run(
            () -> {
                boolean isRed = false;
                var alliance = DriverStation.getAlliance();
                if (alliance.isPresent() && alliance.get() == Alliance.Red) {
                    isRed = true;
                }

                Pose2d currentPose = drive.getPose();
                Translation2d targetCenter = isRed
                    ? new Translation2d(Constants.Layout.FIELD_LENGTH_METERS - blueCenterPosition.getX(), Constants.Layout.FIELD_WIDTH_METERS-blueCenterPosition.getY())
                    : blueCenterPosition;

                double distanceToTarget = currentPose.getTranslation().getDistance(targetCenter);

                double bestPitch = Constants.VisionConfig.distanceToPitchMap.get(distanceToTarget) + Constants.ShootingTrim.pitchOffset;
                double bestSpeed = Constants.VisionConfig.distanceToSpeedMap.get(distanceToTarget) + Constants.ShootingTrim.speedOffset;

                ChassisSpeeds robotRelativeSpeeds = drive.getRobotRelativeSpeeds();
                double cos = currentPose.getRotation().getCos();
                double sin = currentPose.getRotation().getSin();
                double fieldVx = robotRelativeSpeeds.vxMetersPerSecond * cos - robotRelativeSpeeds.vyMetersPerSecond * sin;
                double fieldVy = robotRelativeSpeeds.vxMetersPerSecond * sin + robotRelativeSpeeds.vyMetersPerSecond * cos;
                double dx = targetCenter.getX() - currentPose.getX();
                double dy = targetCenter.getY() - currentPose.getY();
                double dist = Math.hypot(dx, dy);
                double ux = dist > 1e-6 ? dx / dist : 0.0;
                double uy = dist > 1e-6 ? dy / dist : 0.0;
                double radialSpeed = fieldVx * ux + fieldVy * uy;

                double pitchLead = MathUtil.clamp(
                    -PITCH_LEAD_RAD_PER_MPS * radialSpeed,
                    -MAX_PITCH_LEAD_RAD,
                    MAX_PITCH_LEAD_RAD
                );
                double targetPitch = bestPitch - pitchLead;

                launcher.setFrictionWheelVelocity(bestSpeed);
                launcher.setAngleToTarget(targetPitch);
                launcher.setFeederVelocity(
                    warmupTimer.hasElapsed(warmupSeconds)
                        ? Constants.LauncherConfig.FeederSpeed
                        : 0
                );

                SmartDashboard.putNumber("AutoScore/Distance_Meters", distanceToTarget);
                SmartDashboard.putNumber("AutoScore/Target_Pitch", bestPitch);
                SmartDashboard.putNumber("AutoScore/Target_Speed", bestSpeed);
                SmartDashboard.putNumber("AutoScore/PitchLead", pitchLead);
                SmartDashboard.putNumber("AutoScore/RadialSpeed", radialSpeed);
            },
            launcher
        );

        Command transportStream = Commands.sequence(
            Commands.waitSeconds(warmupSeconds),
            Commands.run(
                () -> transport.setTransportVelocity(Constants.TransportConfig.TransportSpeed),
                transport
            )
        );

        Command intakeStream = Commands.sequence(
            Commands.waitSeconds(warmupSeconds),
            Commands.runOnce(() -> {
                intake.setIntakeMotorVelocity(Constants.IntakeConfig.IntakeVelocity);
                intake.setSupportMotorVelocity(Constants.IntakeConfig.SupportVelocity);
            }, intake),
            intake.IntakeSwingSingleCommand().repeatedly()
        );

        return Commands.parallel(
            launcherStream,
            transportStream,
            intakeStream
        ).beforeStarting(() -> {
            warmupTimer.reset();
            warmupTimer.start();
        }).finallyDo((interrupted) -> {
            warmupTimer.stop();
            launcher.setFrictionWheelVelocity(0);
            launcher.setFeederVelocity(0);
            launcher.setAngleVoltage(0);
            transport.setTransportVelocity(0);
            intake.setIntakeMotorVelocity(0);
            intake.setSupportMotorVelocity(0);
            intake.applyIntakePitchMotorNeutral();
        });
    }

    /**
     * 中场盲射 (Feed) 专用的快速发射命令
     * 特点：无视目标位置，无需复杂的视觉距离计算，使用固定角度和射速。
     * 极短预热时间 (0.3s) 保证跑打的流畅性，且不接管底盘控制。
     */
    public static Command createDynamicFeedCommand(
        Intake intake,
        Launcher launcher,
        Transport transport,
        double feedSpeed,
        double feedAngle,
        boolean needSwing
    ) {
        // 极短的预热时间，专为跑打和 Feed 设计 (0.3 秒足够摩擦轮达到较高转速)
        double fastWarmupTime = 0.3; 

        // --- 1. Launcher 的逻辑流 (极短预热 -> 发射) ---
        Command launcherStream = Commands.sequence(
            // 第一阶段：极速预热 (摩擦轮转，Feeder停，调整推杆角度)
            Commands.parallel(
                Commands.run(
                    () -> {
                        launcher.setFrictionWheelVelocity(feedSpeed);
                        launcher.setFeederVelocity(0);
                    }
                ).alongWith(launcher.AdjustAngleToPositionCommand(feedAngle))   
            ).withTimeout(fastWarmupTime), // 极短时间后立刻进入发射

            // 第二阶段：发射 (摩擦轮转，Feeder转)
            Commands.run(
                () -> {
                    launcher.setFrictionWheelVelocity(feedSpeed);
                    launcher.setFeederVelocity(Constants.LauncherConfig.FeederSpeed);
                }, launcher
            )
        );

        // --- 2. Transport 的逻辑流 ---
        Command transportStream = Commands.sequence(
            Commands.waitSeconds(fastWarmupTime), // 等待极短预热
            Commands.run(
                () -> transport.setTransportVelocity(Constants.TransportConfig.TransportSpeed),
                transport
            )
        );

        // --- 3. Intake 的逻辑流 ---
        // 将单纯转动电机的逻辑提取出来
        Command runIntakeMotors = Commands.run(() -> {
            intake.setIntakeMotorVelocity(Constants.IntakeConfig.IntakeVelocity);
            intake.setSupportMotorVelocity(Constants.IntakeConfig.SupportVelocity);
        });

        Command intakeStream = Commands.sequence(
            Commands.waitSeconds(fastWarmupTime), // 等待极短预热
            // 根据 needSwing 参数动态决定执行哪个：
            // 如果为 true，则并行执行电机转动 + 摆动；如果为 false，则只执行电机转动
            needSwing 
                ? Commands.parallel(intake.IntakeFeedingSwingSingleCommand().repeatedly(), runIntakeMotors)
                : runIntakeMotors 
        );

        // --- 组合所有流 ---
        return Commands.parallel(
            launcherStream,
            transportStream,
            intakeStream
        )
        // 中断或结束时立刻清理所有电机
        .finallyDo((interrupted) -> {
            launcher.setFrictionWheelVelocity(0);
            launcher.setFeederVelocity(0);
            launcher.setAngleVoltage(0);
            transport.setTransportVelocity(0);
            // intake.setIntakeMotorVelocity(0);
            intake.setSupportMotorVelocity(0);
            intake.applyIntakePitchMotorNeutral();
        });
    }

    public static Command createAutoDynamicShootingCommand(
        CommandSwerveDrivetrain drive,
        Intake intake,
        Launcher launcher,
        Transport transport,
        Translation2d blueCenterPosition
    ) {
        double warmupSeconds = 4.0;
        Timer warmupTimer = new Timer();

        Command launcherStream = Commands.run(
            () -> {
                boolean isRed = false;
                var alliance = DriverStation.getAlliance();
                if (alliance.isPresent() && alliance.get() == Alliance.Red) {
                    isRed = true;
                }

                Pose2d currentPose = drive.getPose();
                Translation2d targetCenter = isRed
                    ? new Translation2d(Constants.Layout.FIELD_LENGTH_METERS - blueCenterPosition.getX(), Constants.Layout.FIELD_WIDTH_METERS-blueCenterPosition.getY())
                    : blueCenterPosition;

                double distanceToTarget = currentPose.getTranslation().getDistance(targetCenter);

                double bestPitch = Constants.VisionConfig.distanceToPitchMap.get(distanceToTarget) + Constants.ShootingTrim.pitchOffset;
                double bestSpeed = Constants.VisionConfig.distanceToSpeedMap.get(distanceToTarget) + Constants.ShootingTrim.speedOffset;

                ChassisSpeeds robotRelativeSpeeds = drive.getRobotRelativeSpeeds();
                double cos = currentPose.getRotation().getCos();
                double sin = currentPose.getRotation().getSin();
                double fieldVx = robotRelativeSpeeds.vxMetersPerSecond * cos - robotRelativeSpeeds.vyMetersPerSecond * sin;
                double fieldVy = robotRelativeSpeeds.vxMetersPerSecond * sin + robotRelativeSpeeds.vyMetersPerSecond * cos;
                double dx = targetCenter.getX() - currentPose.getX();
                double dy = targetCenter.getY() - currentPose.getY();
                double dist = Math.hypot(dx, dy);
                double ux = dist > 1e-6 ? dx / dist : 0.0;
                double uy = dist > 1e-6 ? dy / dist : 0.0;
                double radialSpeed = fieldVx * ux + fieldVy * uy;

                double pitchLead = MathUtil.clamp(
                    -PITCH_LEAD_RAD_PER_MPS * radialSpeed,
                    -MAX_PITCH_LEAD_RAD,
                    MAX_PITCH_LEAD_RAD
                );
                double targetPitch = bestPitch - pitchLead;

                launcher.setFrictionWheelVelocity(bestSpeed);
                launcher.setAngleToTarget(targetPitch);
                launcher.setFeederVelocity(
                    warmupTimer.hasElapsed(warmupSeconds)
                        ? Constants.LauncherConfig.FeederSpeed
                        : 0
                );

                SmartDashboard.putNumber("AutoScore/Distance_Meters", distanceToTarget);
                SmartDashboard.putNumber("AutoScore/Target_Pitch", bestPitch);
                SmartDashboard.putNumber("AutoScore/Target_Speed", bestSpeed);
                SmartDashboard.putNumber("AutoScore/PitchLead", pitchLead);
                SmartDashboard.putNumber("AutoScore/RadialSpeed", radialSpeed);
            },
            launcher
        );

        Command transportStream = Commands.sequence(
            Commands.waitSeconds(warmupSeconds),
            Commands.run(
                () -> transport.setTransportVelocity(Constants.TransportConfig.TransportSpeed),
                transport
            )
        );

        Command intakeStream = Commands.sequence(
            Commands.waitSeconds(warmupSeconds),
            Commands.runOnce(() -> {
                intake.setIntakeMotorVelocity(Constants.IntakeConfig.IntakeVelocity);
                intake.setSupportMotorVelocity(Constants.IntakeConfig.SupportVelocity);
            }, intake),
            intake.IntakeSwingSingleCommand().repeatedly()
        );

        return Commands.parallel(
            launcherStream,
            transportStream,
            intakeStream
        ).beforeStarting(() -> {
            warmupTimer.reset();
            warmupTimer.start();
        }).finallyDo((interrupted) -> {
            warmupTimer.stop();
            launcher.setFrictionWheelVelocity(0);
            launcher.setFeederVelocity(0);
            launcher.setAngleVoltage(0);
            transport.setTransportVelocity(0);
            // intake.setIntakeMotorVelocity(0);
            intake.setSupportMotorVelocity(0);
            intake.applyIntakePitchMotorNeutral();
        });
    }
}


