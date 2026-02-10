package frc.robot.commands;

import java.util.List;
import java.util.Set;
import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.CommandSwerveDrivetrain;

public class MagicSequencingCommand {

    // 标准 FRC 场地长度 (米)，用于红蓝联盟坐标镜像
    private static final double FIELD_LENGTH_METERS = 16.54099;

    /**
     * 原有的方法：到 Hub 对位但不进行得分
     */
    public static final Command magicScoreNoScoreHub(CommandSwerveDrivetrain drive, Supplier<Pose2d> hubPosition) {
        return drive.pathfindToPose(hubPosition.get())//先规划路径
                .andThen(drive.translateToPositionWithPID(hubPosition.get()));
    }

    public static final Command magicScoreNoScoreHubOnlyPID(CommandSwerveDrivetrain drive, Supplier<Pose2d> hubPosition) {
        return drive.translateToPositionWithPID(hubPosition.get());
    }

    /**
     * 新增方法：根据当前位置，自动选择最近的硬编码点，并面向中心点
     * 
     * @param drive 底盘子系统
     * @param blueScoringPositions 蓝色联盟视角下的候选得分点列表 (Translation2d)
     * @param blueCenterPosition 蓝色联盟视角下的中心目标点 (Hub/Reef中心)，用于计算朝向
     * @return 一个动态命令，执行时会自动计算最近点并规划路径
     */
    public static Command magicRunToClosestHardcodedPose(
            CommandSwerveDrivetrain drive, 
            List<Translation2d> blueScoringPositions, 
            Translation2d blueCenterPosition) {
        
        // 使用 Commands.defer 确保逻辑在命令运行时才执行（因为那时才知道机器人在哪）
        return Commands.defer(() -> {
            
            // 1. 获取联盟颜色
            boolean isRed = false;
            var alliance = DriverStation.getAlliance();
            if (alliance.isPresent() && alliance.get() == Alliance.Red) {
                isRed = true;
            }

            // 2. 获取当前机器人位置
            Pose2d currentPose = drive.getPose();
            
            // 3. 寻找最近的目标点
            Translation2d bestPoint = null;
            double minDistance = Double.MAX_VALUE;

            for (Translation2d point : blueScoringPositions) {
                // 如果是红方，进行场地镜像转换 (X轴翻转)
                Translation2d targetPointToCheck = isRed ? 
                    new Translation2d(FIELD_LENGTH_METERS - point.getX(), point.getY()) : 
                    point;
                
                double distance = currentPose.getTranslation().getDistance(targetPointToCheck);
                if (distance < minDistance) {
                    minDistance = distance;
                    bestPoint = targetPointToCheck;
                }
            }

            // 如果列表为空或出错，原地不动
            if (bestPoint == null) {
                return Commands.none();
            }

            // 4. 计算朝向：从该点(bestPoint) 面向 中心点(centerPosition)
            Translation2d targetCenter = isRed ? 
                new Translation2d(FIELD_LENGTH_METERS - blueCenterPosition.getX(), blueCenterPosition.getY()) : 
                blueCenterPosition;

            double dx = targetCenter.getX() - bestPoint.getX();
            double dy = targetCenter.getY() - bestPoint.getY();
            Rotation2d targetRotation = new Rotation2d(Math.atan2(dy, dx));

            Pose2d targetPose = new Pose2d(bestPoint, targetRotation);

            // 5. 生成路径规划命令 (复用你现有的 pathfindToPose)
            // 你可以根据需求选择只用 pathfind，或者 pathfind + PID
            return drive.pathfindToPose(targetPose)
                   .andThen(drive.translateToPositionWithPID(targetPose)).withTimeout(3.0);

        }, Set.of(drive)); // 声明 subsystem 依赖
    }
}