package frc.robot.commands;

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
import frc.robot.Constants;

public class MagicSequencingCommand {

    /**
     * Pathfinds to the target fixed position, facing the hub.
     * Times out at 5s to prevent getting stuck.
     */
    public static Command runToClosestPosition(
            int position_index,
            CommandSwerveDrivetrain drive,
            double[][] pointsParamsTable,
            Translation2d blueCenterPosition) {

        return Commands.defer(() -> {
            boolean isRed = DriverStation.getAlliance()
                .map(a -> a == Alliance.Red).orElse(false);

            Pose2d currentPose = drive.getPose();

            Translation2d point = new Translation2d(
                pointsParamsTable[position_index][0],
                pointsParamsTable[position_index][1]
            );

            Translation2d targetPoint = isRed
                ? new Translation2d(
                    Constants.Layout.FIELD_LENGTH_METERS - point.getX(),
                    Constants.Layout.FIELD_WIDTH_METERS - point.getY())
                : point;

            if (targetPoint == null) {
                return Commands.none();
            }

            Translation2d targetCenter = isRed
                ? new Translation2d(
                    Constants.Layout.FIELD_LENGTH_METERS - blueCenterPosition.getX(),
                    Constants.Layout.FIELD_WIDTH_METERS - blueCenterPosition.getY())
                : blueCenterPosition;

            double dx = targetCenter.getX() - targetPoint.getX();
            double dy = targetCenter.getY() - targetPoint.getY();
            Rotation2d targetRotation = new Rotation2d(Math.atan2(dy, dx));
            Pose2d targetPose = new Pose2d(targetPoint, targetRotation);

            return drive.translateToPositionWithPID(targetPose).withTimeout(5.0);

        }, Set.of(drive));
    }

    /**
     * Rotates in place to face the hub, then shoots based on distance lookup.
     */
    public static Command createAnyPointAutoScoreCommand(
        CommandSwerveDrivetrain drive,
        Intake intakeSubsystem,
        Launcher launcher,
        Translation2d blueCenterPosition
    ) {
        return Commands.defer(() -> {
            boolean isRed = DriverStation.getAlliance()
                .map(a -> a == Alliance.Red).orElse(false);

            Pose2d currentPose = drive.getPose();
            Translation2d targetCenter = isRed
                ? new Translation2d(
                    Constants.Layout.FIELD_LENGTH_METERS - blueCenterPosition.getX(),
                    Constants.Layout.FIELD_WIDTH_METERS - blueCenterPosition.getY())
                : blueCenterPosition;

            double distanceToTarget = currentPose.getTranslation().getDistance(targetCenter);
            double bestPitch = Constants.VisionConfig.distanceToPitchMap.get(distanceToTarget);
            double bestSpeed = Constants.VisionConfig.distanceToSpeedMap.get(distanceToTarget);

            double dx = targetCenter.getX() - currentPose.getX();
            double dy = targetCenter.getY() - currentPose.getY();
            Rotation2d targetHeading = new Rotation2d(Math.atan2(dy, dx));

            launcher.setTargetDistance(distanceToTarget);
            SmartDashboard.putNumber("AutoScore/Distance_Meters", distanceToTarget);
            SmartDashboard.putNumber("AutoScore/Target_Pitch", bestPitch);
            SmartDashboard.putNumber("AutoScore/Target_Speed", bestSpeed);

            return Commands.sequence(
                Commands.parallel(
                    Commands.run(() -> {
                        launcher.setFrictionWheelVelocity(bestSpeed);
                        launcher.setAngleToTarget(bestPitch);
                    }, launcher),
                    turnToPosition(drive, blueCenterPosition)
                ).until(() -> launcher.isFrictionWheelReady()
                           && launcher.isAngleAtPosition(bestPitch)
                           && drive.isAtHeading(targetHeading))
                 .withTimeout(2.5),
                ShootingCommand.createAutoShootingCommand(intakeSubsystem, launcher, bestSpeed)
            );
        }, Set.of(drive, intakeSubsystem, launcher));
    }

    /**
     * Rotates in place to face the hub center without moving position.
     */
    public static Command turnToPosition(
            CommandSwerveDrivetrain drive,
            Translation2d blueCenterPosition) {

        return Commands.defer(() -> {
            boolean isRed = DriverStation.getAlliance()
                .map(a -> a == Alliance.Red).orElse(false);

            Pose2d currentPose = drive.getPose();
            Translation2d currentPosition = new Translation2d(currentPose.getX(), currentPose.getY());

            Translation2d targetCenter = isRed
                ? new Translation2d(
                    Constants.Layout.FIELD_LENGTH_METERS - blueCenterPosition.getX(),
                    Constants.Layout.FIELD_WIDTH_METERS - blueCenterPosition.getY())
                : blueCenterPosition;

            double dx = targetCenter.getX() - currentPose.getX();
            double dy = targetCenter.getY() - currentPose.getY();
            Rotation2d targetRotation = new Rotation2d(Math.atan2(dy, dx));
            Pose2d targetPose = new Pose2d(currentPosition, targetRotation);

            SmartDashboard.putString("AutoScore/TargetPose", targetPose.toString());
            return drive.translateToRotationWithPID(targetPose).withTimeout(2.0);

        }, Set.of(drive));
    }

    /**
     * Moves to the fixed scoring position, then shoots.
     * Uses standard (slower) path — more reliable for qualifying.
     */
    public static Command createFixedPointAutoScoreCommand(
        int position_index,
        CommandSwerveDrivetrain drive,
        Intake intakeSubsystem,
        Launcher launcher,
        Translation2d blueCenterPosition,
        double[][] pointsParamsTable
    ) {
        return Commands.defer(() -> {
            boolean isRed = DriverStation.getAlliance().map(a -> a == Alliance.Red).orElse(false);
            double[] currentParams = pointsParamsTable[position_index];
            double launch_angle = currentParams[2] + Constants.ShootingTrim.pitchOffset;
            double frictionWheelLaunchSpeed = currentParams[3] + Constants.ShootingTrim.speedOffset;

            Translation2d fixedPoint = new Translation2d(currentParams[0], currentParams[1]);
            Translation2d hubCenter = isRed
                ? new Translation2d(
                    Constants.Layout.FIELD_LENGTH_METERS - blueCenterPosition.getX(),
                    Constants.Layout.FIELD_WIDTH_METERS - blueCenterPosition.getY())
                : blueCenterPosition;
            launcher.setTargetDistance(fixedPoint.getDistance(hubCenter));

            return Commands.sequence(
                Commands.deadline(
                    runToClosestPosition(position_index, drive, pointsParamsTable, blueCenterPosition),
                    Commands.run(() -> {
                        launcher.setFrictionWheelVelocity(frictionWheelLaunchSpeed);
                        launcher.setAngleToTarget(launch_angle);
                    }, launcher)
                ),
                Commands.waitUntil(() -> launcher.isFrictionWheelReady() && launcher.isAngleAtPosition(launch_angle))
                    .withTimeout(0.5),
                ShootingCommand.createAutoShootingCommand(intakeSubsystem, launcher, frictionWheelLaunchSpeed)
            );
        }, Set.of(drive, intakeSubsystem, launcher));
    }

    /**
     * Starts shooting immediately while simultaneously driving to position (fast variant).
     * Assumes friction wheels are already pre-warmed by a prior WarmUp named command.
     */
    public static Command createFastFixedPointAutoScoreCommand(
        int position_index,
        CommandSwerveDrivetrain drive,
        Intake intakeSubsystem,
        Launcher launcher,
        Translation2d blueCenterPosition,
        double[][] pointsParamsTable
    ) {
        return Commands.defer(() -> {
            boolean isRed = DriverStation.getAlliance().map(a -> a == Alliance.Red).orElse(false);
            double[] currentParams = pointsParamsTable[position_index];
            double frictionWheelLaunchSpeed = currentParams[3] + Constants.ShootingTrim.speedOffset;

            Translation2d fixedPoint = new Translation2d(currentParams[0], currentParams[1]);
            Translation2d hubCenter = isRed
                ? new Translation2d(
                    Constants.Layout.FIELD_LENGTH_METERS - blueCenterPosition.getX(),
                    Constants.Layout.FIELD_WIDTH_METERS - blueCenterPosition.getY())
                : blueCenterPosition;
            launcher.setTargetDistance(fixedPoint.getDistance(hubCenter));

            return Commands.parallel(
                ShootingCommand.createAutoShootingCommand(intakeSubsystem, launcher, frictionWheelLaunchSpeed),
                runToClosestPosition(position_index, drive, pointsParamsTable, blueCenterPosition)
            );
        }, Set.of(drive, intakeSubsystem, launcher));
    }
}

