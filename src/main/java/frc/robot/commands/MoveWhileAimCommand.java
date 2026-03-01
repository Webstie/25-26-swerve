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
import frc.robot.Constants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Launcher;
import frc.robot.subsystems.Transport;
import frc.robot.util.MathUtils;

public class MoveWhileAimCommand {
    private static final double FIELD_LENGTH_METERS = 16.54099;

    public static Command create(
        CommandSwerveDrivetrain drive,
        DoubleSupplier xVelocityMps,
        DoubleSupplier yVelocityMps,
        double maxRotRateRadPerSec,
        Translation2d blueCenterPosition
    ) {
        PIDController rotationController = new PIDController(6.0, 0.0, 0.2);
        rotationController.enableContinuousInput(-Math.PI, Math.PI);
        rotationController.setTolerance(Units.degreesToRadians(Constants.VisionConfig.ANGLE_TOLERANCE_DEGREES));

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
