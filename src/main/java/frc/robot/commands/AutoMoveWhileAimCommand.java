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

// Auto-phase mobile shooting (currently not working well).
// Future: integrate Kalman filter and vision for smarter dynamic aiming.
// Controls heading only — does not trigger launching.
public class AutoMoveWhileAimCommand {
    private static final double LEAD_GAIN_RAD_PER_MPS = 0.5;
    private static final double MAX_LEAD_RAD = Units.degreesToRadians(15.0);
    private static final double MIN_TARGET_DISTANCE_METERS = 0.05;

    public static Command create(
        CommandSwerveDrivetrain drive,
        Translation2d blueCenterPosition
    ) {
        // Use Commands.startEnd to attach aiming logic on start and detach on end
        return Commands.startEnd(
            () -> {
                // Enable PathPlanner rotation target override.
                // PathPlanner will call this supplier each loop, replacing the path's original heading.
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

                    // Return the calculated lead-compensated target heading
                    return Optional.of(new Rotation2d(targetHeadingRad));
                });
            },
            () -> {
                // When the command ends (timeout or shot complete), clear the rotation override.
                // Drivetrain will automatically resume the original PathPlanner heading.
                PPHolonomicDriveController.setRotationTargetOverride(null);
            }
        );
        // Important: do NOT add .addRequirements(drive) here!
        // This command only modifies a PathPlanner static parameter; actual driving is still handled by the active path.
    }
}
