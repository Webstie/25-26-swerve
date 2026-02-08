package frc.robot.commands;
import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CommandSwerveDrivetrain;

/**
 * MagicSequencing 类
 * 该类封装了多种比赛中常用的“复合动作”命令组合（驾驶+机械臂+抓取/放置逻辑），
 * 通过 WPILib Command API 将移动、机械臂动作和进出料同步执行。
 */

public class MagicSequencingCommand {

    /**
     * 自动路径规划到指定 Reef，并放置 Coral
     * @param drive 底盘驱动子系统
     * @param armistice 机械臂+升降机构子系统
     * @param coral 珊瑚操纵机构
     * @param reefPosition 目标 Reef 的位姿（Pose2d）
     * @param scorePosition 目标放置位置枚举（例如 Cora_L4、BARGE）
     */

    /**
     * 到 Hub 对位但不进行得分，仅保持位置
     */
    public static final Command magicScoreNoScoreHub(CommandSwerveDrivetrain drive,Supplier<Pose2d> hubPosition) {
        return drive.pathfindToPose(hubPosition.get())//先规划路径
                .andThen(drive.translateToPositionWithPID(hubPosition.get()));
    }

    public static final Command magicScoreNoScoreReefOnlyPID(CommandSwerveDrivetrain drive, Supplier<Pose2d> reefPosition) {
        return drive.translateToPositionWithPID(reefPosition.get()); // 直接PID 移动
    }

}
