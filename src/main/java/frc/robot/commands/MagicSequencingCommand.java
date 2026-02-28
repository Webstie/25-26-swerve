package frc.robot.commands;

import java.util.List;
import java.util.Set;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Launcher;
import frc.robot.subsystems.Transport;
import frc.robot.Constants;

public class MagicSequencingCommand {

    //FRC 场地长度 (米)，用于红蓝联盟坐标镜像
    private static final double FIELD_LENGTH_METERS = 16.54099;

    /**
     * 根据当前位置，并面向Hub,瞄准后开始射击
     * 
     * @param drive 底盘子系统
     * @param blueScoringPositions 蓝色联盟视角下的候选得分点列表 (Translation2d)
     * @param blueCenterPosition 蓝色联盟视角下的中心目标点 (Hub/Reef中心)，用于计算朝向
     * @return 一个动态命令，执行时会自动计算最近点并规划路径
     */

    //********************************************************移动到固定点位自瞄发射********************************************************** */
    public static Command magicRunToClosestHardcodedPose(
            int position_index,
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
            
            // // 3. 寻找最近的目标点
            // Translation2d bestPoint = null;
            // double minDistance = Double.MAX_VALUE;

            // for (Translation2d point : blueScoringPositions) {
            //     // 如果是红方，进行场地镜像转换 (X轴翻转)
            //     Translation2d targetPointToCheck = isRed ? 
            //         new Translation2d(FIELD_LENGTH_METERS - point.getX(), point.getY()) : 
            //         point;
                
            //     double distance = currentPose.getTranslation().getDistance(targetPointToCheck);
            //     if (distance < minDistance) {
            //         minDistance = distance;
            //         bestPoint = targetPointToCheck;
            //     }
            // }

            // 3. 寻找最近的目标点
            Translation2d bestPoint = null;
            double minDistance = Double.MAX_VALUE;

            Translation2d point = blueScoringPositions.get(position_index);

            Translation2d targetPointToCheck = isRed ? 
                new Translation2d(FIELD_LENGTH_METERS - point.getX(), point.getY()) : 
                point;
            
            double distance = currentPose.getTranslation().getDistance(targetPointToCheck);
            if (distance < minDistance) {
                minDistance = distance;
                bestPoint = targetPointToCheck;
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
                   .andThen(drive.translateToPositionWithPID(targetPose)).withTimeout(5.0);// 设置超时为5秒，防止卡死

        }, Set.of(drive)); // 声明 subsystem 依赖
    }

    /**
     * 创建一个顺序命令，先移动到最近的硬编码得分点，然后开始射击
     * 
     * @param drive 底盘子系统
     * @param intakeSubsystem Intake子系统实例
     * @param launcher Shooter子系统实例
     * @param transportSubsystem Transport子系统实例
     * @param blueScoringPositions 蓝色联盟视角下的候选得分点列表 (Translation2d)
     * @param blueCenterPosition 蓝色联盟视角下的中心目标点 (Hub/Reef中心)，用于计算朝向
     * @param frictionWheelLaunchSpeed 发射要的摩擦轮转速
     * @param launch_angle 发射要的角度
     * @return 一个Command对象，表示顺序执行的自动得分命令
     */
    public static Command createSequentialAutoScoreCommand(
        int position_index,
        CommandSwerveDrivetrain drive,
        Intake intakeSubsystem,
        Launcher launcher,
        Transport transportSubsystem,
        List<Translation2d> blueScoringPositions,
        Translation2d blueCenterPosition,
        double frictionWheelLaunchSpeed,
        double launch_angle

    ) {
        return Commands.defer(() -> {
            return Commands.sequence(
                // 第一阶段：移动到目标位置
                Commands.parallel(     
                    Commands.runOnce(()->launcher.setFrictionWheelVelocity(frictionWheelLaunchSpeed)),//预热       
                    launcher.AdjustAngleToPositionCommand(launch_angle),
                    MagicSequencingCommand.magicRunToClosestHardcodedPose(position_index,drive, blueScoringPositions, blueCenterPosition)//移动
                
                ),
                // 第二阶段：到达位置后开始射击
                ShootingCommand.createAutoShootingCommand(intakeSubsystem, launcher, transportSubsystem, frictionWheelLaunchSpeed));
        }, Set.of(drive, intakeSubsystem, launcher, transportSubsystem));
    }


//*******************************************************************任意点位原地自瞄发射***************************************************************
    public static Command turn2PositionCommand(
            CommandSwerveDrivetrain drive, 
            Translation2d blueCenterPosition) {
        
        // 使用 Commands.defer 确保逻辑在命令运行时才执行
        return Commands.defer(() -> {
            
            // 1. 获取联盟颜色
            boolean isRed = false;
            var alliance = DriverStation.getAlliance();
            if (alliance.isPresent() && alliance.get() == Alliance.Red) {
                isRed = true;
            }

            // 2. 获取当前机器人位置
            Pose2d currentPose = drive.getPose();
            Translation2d currentPosition = new Translation2d(currentPose.getX(), currentPose.getY());

            // 4. 计算朝向：从该点(bestPoint) 面向 中心点(centerPosition)
            Translation2d targetCenter = isRed ? 
                new Translation2d(FIELD_LENGTH_METERS - blueCenterPosition.getX(), blueCenterPosition.getY()) : 
                blueCenterPosition;
            
            double dx = targetCenter.getX() - currentPose.getX();
            double dy = targetCenter.getY() - currentPose.getY();
            Rotation2d targetRotation = new Rotation2d(Math.atan2(dy, dx));

            Pose2d targetPose = new Pose2d(currentPosition, targetRotation);//直接给当前位置，保持位置不动，直接瞄准

            // 5. 生成路径规划命令 (复用你现有的 pathfindToPose)
            // 你可以根据需求选择只用 pathfind，或者 pathfind + PID
            return drive.pathfindToPose(targetPose)
                   .andThen(drive.translateToRotationWithPID(targetPose)).withTimeout(2.0);// 设置超时为5秒，防止卡死,只动角度

        }, Set.of(drive)); // 声明 subsystem 依赖

    }

    //对于pitch电推杆的角度调整目前放在了command中，也可以考虑放在某个子系统的periodic里面实时计算，然后拿结果
    public static Command createAutoTurnScoreCommand(
        CommandSwerveDrivetrain drive,
        Intake intakeSubsystem,
        Launcher launcher,
        Transport transportSubsystem,
        List<Translation2d> blueScoringPositions,
        Translation2d blueCenterPosition
    ) {
        return Commands.defer(() -> {
            // 1. 获取联盟颜色并确定目标中心点坐标
            boolean isRed = false;
            var alliance = DriverStation.getAlliance();
            if (alliance.isPresent() && alliance.get() == Alliance.Red) {
                isRed = true;
            }

            // 2. 获取当前机器人位置
            Pose2d currentPose = drive.getPose();

            // 3. 计算目标点的绝对坐标 (处理红蓝联盟镜像)
            // 假设 Constants.Field.FIELD_LENGTH_METERS 是场地的总长度 (约16.54米)
            // 如果你还没有这个常量，请替换为具体的数值，例如 16.54175
            double fieldLength = 16.54175; 
            
            Translation2d targetCenter = isRed ? 
                new Translation2d(fieldLength - blueCenterPosition.getX(), blueCenterPosition.getY()) : 
                blueCenterPosition;

            // 4. 计算欧氏距离 (Euclidean Distance)
            double distanceToTarget = currentPose.getTranslation().getDistance(targetCenter);

            // 5. 查表逻辑 (最近邻查找)
            double bestPitch = 0.0;
            double bestSpeed = 0.0;

            double[][] table = Constants.VisionConfig.DISTANCE_PARAMS_TABLE;
            int lastIndex = table.length - 1;

            double[] lower = table[Math.max(0, lastIndex - 1)];
            double[] upper = table[lastIndex];
            for (int i = 0; i < lastIndex; i++) {
                double d0 = table[i][0];
                double d1 = table[i + 1][0];
                if (distanceToTarget <= d0) {
                    lower = table[i];
                    upper = table[Math.min(i + 1, lastIndex)];
                    break;
                }
                if (distanceToTarget <= d1) {
                    lower = table[i];
                    upper = table[i + 1];
                    break;
                }
            }

            double x0 = lower[0];
            double x1 = upper[0];

            double pitchSlope = (x1 - x0) == 0.0 ? 0.0 : (upper[1] - lower[1]) / (x1 - x0);
            double speedSlope = (x1 - x0) == 0.0 ? 0.0 : (upper[2] - lower[2]) / (x1 - x0);

            bestPitch = lower[1] + pitchSlope * (distanceToTarget - x0);
            bestSpeed = lower[2] + speedSlope * (distanceToTarget - x0);

            // 遍历新的 DISTANCE_DATA_TABLE
            // for (double[] row : Constants.VisionConfig.DISTANCE_DATA_TABLE) {
            //     double tableDist = row[0];  // 第1列：距离
            //     double tablePitch = row[1]; // 第2列：Pitch
            //     double tableSpeed = row[2]; // 第3列：速度
                
            //     double diff = Math.abs(distanceToTarget - tableDist);
                
            //     if (diff < minDistanceDiff) {
            //         minDistanceDiff = diff;
            //         bestPitch = tablePitch;
            //         bestSpeed = tableSpeed;
            //     }
            // }
            // 遍历新的 DISTANCE_DATA_TABLE
            // bestPitch = Constants.VisionConfig.PitchSlope * distanceToTarget + Constants.VisionConfig.PitchYIntercept;
            // bestSpeed = Constants.VisionConfig.SpeedSlope * distanceToTarget + Constants.VisionConfig.SpeedYIntercept;


            // 打印调试信息 (不再使用 System.out.println)
            SmartDashboard.putNumber("AutoScore/Distance_Meters", distanceToTarget);
            SmartDashboard.putNumber("AutoScore/Target_Pitch", bestPitch);
            SmartDashboard.putNumber("AutoScore/Target_Speed", bestSpeed);

            // 为了在Lambda内部使用，需要 final 变量 (虽然上面的变量已经是实际上的final)
            final double finalSpeed = bestSpeed;
            final double finalPitch = bestPitch;

            // 6. 返回命令序列
            return Commands.sequence(
                // 第一阶段：原地调整瞄准角度 + 预热
                Commands.parallel(
                    // 预热飞轮
                    Commands.runOnce(() -> launcher.setFrictionWheelVelocity(finalSpeed)),
                    // 使用动态计算出的 finalSpeed 设置电推杆角度
                    launcher.AdjustAngleToPositionCommand(finalPitch),
                    // 转向目标
                    MagicSequencingCommand.turn2PositionCommand(drive, blueCenterPosition)
                ),

                // 第二阶段：到达位置后开始射击
                ShootingCommand.createAutoShootingCommand(intakeSubsystem, launcher, transportSubsystem, finalSpeed)
            );
             
        }, Set.of(drive, intakeSubsystem, launcher, transportSubsystem));
    }
}




