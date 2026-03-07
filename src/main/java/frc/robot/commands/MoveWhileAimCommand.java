package frc.robot.commands;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Launcher;
import frc.robot.subsystems.Transport;
import frc.robot.util.MathUtils;

public class MoveWhileAimCommand {
    private static final double FIELD_LENGTH_METERS = 16.54099;
    private static final double LEAD_GAIN_RAD_PER_MPS = 0.5;
    private static final double MAX_LEAD_RAD = Units.degreesToRadians(20.0);
    private static final double MIN_TARGET_DISTANCE_METERS = 0.05;

    public static Command create(
        CommandSwerveDrivetrain drive,
        DoubleSupplier xVelocityMps,
        DoubleSupplier yVelocityMps,
        double maxRotRateRadPerSec,
        Translation2d blueCenterPosition
    ) {
        PIDController rotationController = new PIDController(3.0, 0.0, 0.0);
        rotationController.enableContinuousInput(-Math.PI, Math.PI);
        rotationController.setTolerance(Units.degreesToRadians(10.0));

        SwerveRequest.FieldCentric request = new SwerveRequest.FieldCentric()
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

        return drive.applyRequest(() -> {
            boolean isRed = false;
            var alliance = DriverStation.getAlliance();
            if (alliance.isPresent() && alliance.get() == Alliance.Red) {
                isRed = true;
            }

            Pose2d currentPose = drive.getPose();
            Translation2d targetCenter = isRed
                ? new Translation2d(FIELD_LENGTH_METERS - blueCenterPosition.getX(), blueCenterPosition.getY())
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

            double rotRate = rotationController.calculate(
                currentPose.getRotation().getRadians(),
                targetHeadingRad
            );
            rotRate = MathUtils.clamp(rotRate, -maxRotRateRadPerSec, maxRotRateRadPerSec);

            return request
                .withVelocityX(xVelocityMps.getAsDouble())
                .withVelocityY(yVelocityMps.getAsDouble())
                .withRotationalRate(rotRate);
        }).finallyDo(() -> rotationController.reset());
    }
}
