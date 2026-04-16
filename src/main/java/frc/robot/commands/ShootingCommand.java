package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Launcher;
import frc.robot.subsystems.CommandSwerveDrivetrain;

import frc.robot.Constants;

public class ShootingCommand extends SequentialCommandGroup {
    private static final double PITCH_LEAD_RAD_PER_MPS = 0.025;
    private static final double MAX_PITCH_LEAD_RAD = 0.02;

    /**
     * Computes dynamic shooting parameters and applies them to the launcher.
     * @param fire if true, enables feeder and transport; if false, holds them at 0 (warmup mode)
     * @param latestTargetPitch  shared array[0] updated each tick with current target pitch
     * @param latestTargetHeadingRad shared array[0] updated each tick with current target heading
     */
    private static void updateDynamic(
            CommandSwerveDrivetrain drive, Launcher launcher,
            Translation2d blueCenterPosition,
            double[] latestTargetPitch, double[] latestTargetHeadingRad,
            boolean fire) {

        boolean isRed = DriverStation.getAlliance().map(a -> a == Alliance.Red).orElse(false);
        Pose2d currentPose = drive.getPose();
        Translation2d targetCenter = isRed
            ? new Translation2d(
                Constants.Layout.FIELD_LENGTH_METERS - blueCenterPosition.getX(),
                Constants.Layout.FIELD_WIDTH_METERS - blueCenterPosition.getY())
            : blueCenterPosition;

        double distanceToTarget = currentPose.getTranslation().getDistance(targetCenter);
        double bestPitch = Constants.VisionConfig.distanceToPitchMap.get(distanceToTarget)
            + Constants.ShootingTrim.pitchOffset;
        double bestSpeed = Constants.VisionConfig.distanceToSpeedMap.get(distanceToTarget)
            + Constants.ShootingTrim.speedOffset;

        ChassisSpeeds speeds = drive.getRobotRelativeSpeeds();
        double cos = currentPose.getRotation().getCos();
        double sin = currentPose.getRotation().getSin();
        double fieldVx = speeds.vxMetersPerSecond * cos - speeds.vyMetersPerSecond * sin;
        double fieldVy = speeds.vxMetersPerSecond * sin + speeds.vyMetersPerSecond * cos;
        double dx = targetCenter.getX() - currentPose.getX();
        double dy = targetCenter.getY() - currentPose.getY();
        double dist = Math.hypot(dx, dy);
        double ux = dist > 1e-6 ? dx / dist : 0.0;
        double uy = dist > 1e-6 ? dy / dist : 0.0;
        double radialSpeed = fieldVx * ux + fieldVy * uy;

        double pitchLead = MathUtil.clamp(
            -PITCH_LEAD_RAD_PER_MPS * radialSpeed, -MAX_PITCH_LEAD_RAD, MAX_PITCH_LEAD_RAD);
        double targetPitch = bestPitch - pitchLead;
        Rotation2d targetHeading = new Rotation2d(Math.atan2(dy, dx));

        latestTargetPitch[0] = targetPitch;
        latestTargetHeadingRad[0] = targetHeading.getRadians();

        launcher.setFrictionWheelVelocity(bestSpeed);
        launcher.setAngleToTarget(targetPitch);
        launcher.setFeederVelocity(fire ? Constants.LauncherConfig.FeederSpeed : 0);
        launcher.setTransportVelocity(fire ? Constants.TransportConfig.TransportSpeed : 0);

        SmartDashboard.putNumber("AutoScore/Distance_Meters", distanceToTarget);
        SmartDashboard.putNumber("AutoScore/Target_Pitch", bestPitch);
        SmartDashboard.putNumber("AutoScore/Target_Speed", bestSpeed);
        SmartDashboard.putNumber("AutoScore/PitchLead", pitchLead);
        SmartDashboard.putNumber("AutoScore/RadialSpeed", radialSpeed);
        SmartDashboard.putBoolean("Shoot/WheelReady", launcher.isFrictionWheelReady());
        SmartDashboard.putBoolean("Shoot/HeadingReady", drive.isAtHeading(targetHeading));
        SmartDashboard.putBoolean("Shoot/AngleReady", launcher.isAngleAtPosition(targetPitch));
        SmartDashboard.putBoolean("Shoot/Firing", fire);
    }

    /**
     * Standard shoot: warmup friction wheels, then feed + transport + intake swing.
     * Transport is now managed by Launcher.
     */
    public static Command createShootingCommand(
        Intake intake,
        Launcher launcher,
        double frictionWheelLaunchSpeed,
        double launchAngle
    ) {
        double warmupTime = Constants.LauncherConfig.WarmupSecond;

        Command launcherStream = Commands.sequence(
            Commands.run(() -> {
                launcher.setFrictionWheelVelocity(frictionWheelLaunchSpeed);
                launcher.setAngleToTarget(launchAngle);
                launcher.setFeederVelocity(0);
                launcher.setTransportVelocity(0);
                SmartDashboard.putBoolean("Shoot/WheelReady", launcher.isFrictionWheelReady());
                SmartDashboard.putBoolean("Shoot/AngleReady", launcher.isAngleAtPosition(launchAngle));
            }, launcher)
            .until(() -> launcher.isFrictionWheelReady() && launcher.isAngleAtPosition(launchAngle))
            .withTimeout(warmupTime),
            Commands.run(() -> {
                launcher.setFrictionWheelVelocity(frictionWheelLaunchSpeed);
                launcher.setFeederVelocity(Constants.LauncherConfig.FeederSpeed);
                launcher.setTransportVelocity(Constants.TransportConfig.TransportSpeed);
            }, launcher)
        );

        Command intakeStream = Commands.sequence(
            Commands.waitUntil(() -> launcher.isFrictionWheelReady() && launcher.isAngleAtPosition(launchAngle))
                .withTimeout(warmupTime),
            Commands.parallel(
                intake.progressiveIntakeSwingCommand()
                    .alongWith(Commands.run(() ->
                        intake.setIntakeMotorVelocity(Constants.IntakeConfig.IntakeVelocity)))
            )
        );

        return Commands.parallel(launcherStream, intakeStream)
            .finallyDo((interrupted) -> {
                launcher.setFrictionWheelVelocity(0);
                launcher.setFeederVelocity(0);
                launcher.setTransportVelocity(0);
                intake.setIntakeMotorVelocity(0);
                intake.resetIntakeCounter();
                intake.applyIntakePitchMotorNeutral();
                launcher.setAngleVoltage(0);
            });
    }

    /**
     * Auto shoot: no warmup wait (assumes pre-warmup during path travel).
     */
    public static Command createAutoShootingCommand(
        Intake intake,
        Launcher launcher,
        double frictionWheelLaunchSpeed
    ) {
        Command launcherStream = Commands.run(() -> {
            launcher.setFrictionWheelVelocity(frictionWheelLaunchSpeed);
            launcher.setFeederVelocity(Constants.LauncherConfig.FeederSpeed);
            launcher.setTransportVelocity(Constants.TransportConfig.TransportSpeed);
        }, launcher);

        Command intakeStream = Commands.parallel(
            intake.progressiveIntakeSwingCommand()
                .alongWith(Commands.run(() ->
                    intake.setIntakeMotorVelocity(Constants.IntakeConfig.IntakeVelocity)))
        );

        return Commands.parallel(launcherStream, intakeStream)
            .finallyDo((interrupted) -> {
                launcher.setFrictionWheelVelocity(0);
                launcher.setFeederVelocity(0);
                launcher.setTransportVelocity(0);
                intake.setIntakeMotorVelocity(0);
                intake.applyIntakePitchMotorNeutral();
            });
    }

    /**
     * Dynamic shoot while moving: distance-based pitch/speed lookup with motion compensation.
     * Pass WarmupSecond for teleop, AutoDynamicWarmupSeconds for auto.
     */
    public static Command createDynamicShootingCommand(
        CommandSwerveDrivetrain drive,
        Intake intake,
        Launcher launcher,
        Translation2d blueCenterPosition,
        double warmupSeconds
    ) {
        // Shared state: warmup run writes latest computed values, .until() reads them
        double[] latestTargetPitch = { 0.0 };
        double[] latestTargetHeadingRad = { 0.0 };

        Command launcherWarmup = Commands.run(
            () -> updateDynamic(drive, launcher, blueCenterPosition, latestTargetPitch, latestTargetHeadingRad, false),
            launcher)
        .until(() -> launcher.isFrictionWheelReady()
                  && launcher.isAngleAtPosition(latestTargetPitch[0])
                  && drive.isAtHeading(new Rotation2d(latestTargetHeadingRad[0])))
        .withTimeout(warmupSeconds);

        Command launcherFire = Commands.run(
            () -> updateDynamic(drive, launcher, blueCenterPosition, latestTargetPitch, latestTargetHeadingRad, true),
            launcher);

        Command launcherStream = Commands.sequence(launcherWarmup, launcherFire);

        Command intakeStream = Commands.sequence(
            Commands.waitUntil(() -> launcher.isFrictionWheelReady()
                                  && launcher.isAngleAtPosition(latestTargetPitch[0])
                                  && drive.isAtHeading(new Rotation2d(latestTargetHeadingRad[0])))
                .withTimeout(warmupSeconds),
            Commands.runOnce(() ->
                intake.setIntakeMotorVelocity(Constants.IntakeConfig.IntakeVelocity), intake),
            intake.progressiveIntakeSwingCommand()
        );

        return Commands.parallel(launcherStream, intakeStream)
            .finallyDo((interrupted) -> {
                launcher.setFrictionWheelVelocity(0);
                launcher.setFeederVelocity(0);
                launcher.setAngleVoltage(0);
                launcher.setTransportVelocity(0);
                intake.setIntakeMotorVelocity(0);
                intake.resetIntakeCounter();
                intake.applyIntakePitchMotorNeutral();
            });
    }

    /**
     * Mid-field feed: fixed angle/speed, short warmup, optional intake swing.
     */
    public static Command createDynamicFeedCommand(
        Intake intake,
        Launcher launcher,
        double feedSpeed,
        double feedAngle,
        boolean needSwing
    ) {
        double fastWarmupTime = Constants.LauncherConfig.FastWarmupSeconds;

        Command launcherStream = Commands.sequence(
            Commands.run(() -> {
                launcher.setFrictionWheelVelocity(feedSpeed);
                launcher.setAngleToTarget(feedAngle);
                launcher.setFeederVelocity(0);
                launcher.setTransportVelocity(0);
            }, launcher)
            .until(() -> launcher.isFrictionWheelReady() && launcher.isAngleAtPosition(feedAngle))
            .withTimeout(fastWarmupTime),
            Commands.run(() -> {
                launcher.setFrictionWheelVelocity(feedSpeed);
                launcher.setFeederVelocity(Constants.LauncherConfig.FeederSpeed);
                launcher.setTransportVelocity(Constants.TransportConfig.TransportSpeed);
            }, launcher)
        );

        Command runIntakeMotors = Commands.run(() ->
            intake.setIntakeMotorVelocity(Constants.IntakeConfig.IntakeVelocity));

        Command intakeStream = Commands.sequence(
            Commands.waitUntil(() -> launcher.isFrictionWheelReady() && launcher.isAngleAtPosition(feedAngle))
                .withTimeout(fastWarmupTime),
            needSwing
                ? Commands.parallel(intake.progressiveIntakeSwingCommand(), runIntakeMotors)
                : runIntakeMotors
        );

        return Commands.parallel(launcherStream, intakeStream)
            .finallyDo((interrupted) -> {
                launcher.setFrictionWheelVelocity(0);
                launcher.setFeederVelocity(0);
                launcher.setAngleVoltage(0);
                launcher.setTransportVelocity(0);
                intake.setIntakeMotorVelocity(0);
                intake.applyIntakePitchMotorNeutral();
            });
    }

}
