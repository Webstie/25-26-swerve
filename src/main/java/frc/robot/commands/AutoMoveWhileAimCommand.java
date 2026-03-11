package frc.robot.commands;

import java.util.Optional;

import com.pathplanner.lib.controllers.PPHolonomicDriveController;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.util.MathUtils;

//自动阶段跑打（暂时还不好使），后续考虑结合卡尔曼滤波器和视觉系统做更智能的动态瞄准
public class AutoMoveWhileAimCommand {
    private static final double LEAD_GAIN_RAD_PER_MPS = 0.5;
    private static final double MAX_LEAD_RAD = Units.degreesToRadians(15.0);
    private static final double MIN_TARGET_DISTANCE_METERS = 0.05;

    public static Command create(
        CommandSwerveDrivetrain drive,
        Translation2d blueCenterPosition
    ) {
        // 使用 Commands.startEnd，确保命令启动时挂载瞄准逻辑，结束时卸载
        return Commands.startEnd(
            () -> {
                // 开启 PathPlanner 的旋转目标覆盖 (Rotation Override)
                // PathPlanner 在运行时会自动调用这个 Supplier，以此角度代替原路径中的朝向
                PPHolonomicDriveController.setRotationTargetOverride(() -> {
                    boolean isRed = false;
                    var alliance = DriverStation.getAlliance();
                    if (alliance.isPresent() && alliance.get() == Alliance.Red) {
                        isRed = true;
                    }

                    Pose2d currentPose = drive.getPose();
                    Translation2d targetCenter = isRed
                        ? new Translation2d(Constants.Layout.FIELD_LENGTH_METERS - blueCenterPosition.getX(), Constants.Layout.FIELD_WIDTH_METERS-blueCenterPosition.getY())
                        : blueCenterPosition;

                    double dx = targetCenter.getX() - currentPose.getX();
                    double dy = targetCenter.getY() - currentPose.getY();
                    double targetHeadingRad = Math.atan2(dy, dx);

                    double distance = Math.hypot(dx, dy);
                    if (distance > MIN_TARGET_DISTANCE_METERS) {
                        var robotRelativeSpeeds = drive.getRobotRelativeSpeeds();
                        double cos = currentPose.getRotation().getCos();
                        double sin = currentPose.getRotation().getSin();
                        double fieldVx = robotRelativeSpeeds.vxMetersPerSecond * cos - robotRelativeSpeeds.vyMetersPerSecond * sin;
                        double fieldVy = robotRelativeSpeeds.vxMetersPerSecond * sin + robotRelativeSpeeds.vyMetersPerSecond * cos;

                        double ux = dx / distance;
                        double uy = dy / distance;
                        double lateralUx = -uy;
                        double lateralUy = ux;
                        double lateralSpeed = fieldVx * lateralUx + fieldVy * lateralUy;

                        double leadAngle = MathUtils.clamp(
                            -LEAD_GAIN_RAD_PER_MPS * lateralSpeed,
                            -MAX_LEAD_RAD,
                            MAX_LEAD_RAD
                        );
                        targetHeadingRad += leadAngle;
                        SmartDashboard.putNumber("Aim/LeadAngleDeg", Units.radiansToDegrees(leadAngle));
                        SmartDashboard.putNumber("Aim/LateralSpeed", lateralSpeed);
                    }

                    // 返回计算出的带提前量的目标朝向
                    return Optional.of(new Rotation2d(targetHeadingRad));
                });
            },
            () -> {
                // 当命令结束（例如过了 timeout 时限或发射完毕），清空旋转覆盖
                // 底盘会自动恢复到顺着 PathPlanner 原本设定的方向
                PPHolonomicDriveController.setRotationTargetOverride(null);
            }
        ); 
        // 关键：不要在这里加 .addRequirements(drive) !
        // 这个命令只负责修改 PathPlanner 的静态参数，实际开车的依然是正在跑路径的 PathPlanner
    }
}