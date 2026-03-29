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
import frc.robot.subsystems.CommandSwerveDrivetrain;

import frc.robot.Constants;

public class ShootingCommand extends SequentialCommandGroup {
    private static final double PITCH_LEAD_RAD_PER_MPS = 0.025;
    private static final double MAX_PITCH_LEAD_RAD = 0.02;

    /**
     * Standard shoot: warmup friction wheels, then feed + transport + intake swing.
     * Transport is now managed by Launcher.
     */
    public static Command createShootingCommand(
        Intake intake,
        Launcher launcher,
        double frictionWheelLaunchSpeed,
        double launch_angle
    ) {
        double warmupTime = Constants.LauncherConfig.WarmupSecond;

        Command launcherStream = Commands.sequence(
            Commands.parallel(
                Commands.run(() -> {
                    launcher.setFrictionWheelVelocity(frictionWheelLaunchSpeed);
                    launcher.setFeederVelocity(0);
                })
                .alongWith(launcher.AdjustAngleToPositionCommand(launch_angle))
            ).withTimeout(warmupTime),
            Commands.run(() -> {
                launcher.setFrictionWheelVelocity(frictionWheelLaunchSpeed);
                launcher.setFeederVelocity(Constants.LauncherConfig.FeederSpeed);
            }, launcher)
        );

        Command transportStream = Commands.sequence(
            Commands.waitSeconds(warmupTime),
            Commands.run(() -> launcher.setTransportVelocity(Constants.TransportConfig.TransportSpeed), launcher)
        );

        Command intakeStream = Commands.sequence(
            Commands.waitSeconds(warmupTime),
            Commands.parallel(
                intake.IntakeSwingSingleCommand().repeatedly()
                    .alongWith(Commands.run(() ->
                        intake.setIntakeMotorVelocity(Constants.IntakeConfig.IntakeVelocity)))
            )
        );

        return Commands.parallel(launcherStream, transportStream, intakeStream)
            .finallyDo((interrupted) -> {
                launcher.setFrictionWheelVelocity(0);
                launcher.setFeederVelocity(0);
                launcher.setTransportVelocity(0);
                intake.setIntakeMotorVelocity(0);
                intake.resetIntakeCounter();
                intake.setSupportMotorVelocity(0);
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
        });

        Command transportStream = Commands.run(
            () -> launcher.setTransportVelocity(Constants.TransportConfig.TransportSpeed), launcher);

        Command intakeStream = Commands.parallel(
            intake.IntakeSwingSingleCommand().repeatedly()
                .alongWith(Commands.run(() -> {
                    intake.setIntakeMotorVelocity(Constants.IntakeConfig.IntakeVelocity);
                    intake.setSupportMotorVelocity(Constants.IntakeConfig.SupportVelocity);
                }))
        );

        return Commands.parallel(launcherStream, transportStream, intakeStream)
            .finallyDo((interrupted) -> {
                launcher.setFrictionWheelVelocity(0);
                launcher.setFeederVelocity(0);
                launcher.setTransportVelocity(0);
                intake.setIntakeMotorVelocity(0);
                intake.setSupportMotorVelocity(0);
                intake.applyIntakePitchMotorNeutral();
            });
    }

    /**
     * Dynamic shoot while moving: distance-based pitch/speed lookup with motion compensation.
     */
    public static Command createDynamicShootingCommand(
        CommandSwerveDrivetrain drive,
        Intake intake,
        Launcher launcher,
        Translation2d blueCenterPosition
    ) {
        double warmupSeconds = Constants.LauncherConfig.WarmupSecond;
        Timer warmupTimer = new Timer();

        Command launcherStream = Commands.run(
            () -> {
                boolean isRed = DriverStation.getAlliance()
                    .map(a -> a == Alliance.Red).orElse(false);

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
                    -PITCH_LEAD_RAD_PER_MPS * radialSpeed, -MAX_PITCH_LEAD_RAD, MAX_PITCH_LEAD_RAD);
                double targetPitch = bestPitch - pitchLead;

                launcher.setFrictionWheelVelocity(bestSpeed);
                launcher.setAngleToTarget(targetPitch);
                launcher.setFeederVelocity(
                    warmupTimer.hasElapsed(warmupSeconds) ? Constants.LauncherConfig.FeederSpeed : 0);

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
            Commands.run(() -> launcher.setTransportVelocity(Constants.TransportConfig.TransportSpeed), launcher)
        );

        Command intakeStream = Commands.sequence(
            Commands.waitSeconds(warmupSeconds),
            Commands.runOnce(() -> {
                intake.setIntakeMotorVelocity(Constants.IntakeConfig.IntakeVelocity);
                intake.setSupportMotorVelocity(Constants.IntakeConfig.SupportVelocity);
            }, intake),
            intake.IntakeSwingSingleCommand().repeatedly()
        );

        return Commands.parallel(launcherStream, transportStream, intakeStream)
            .beforeStarting(() -> { warmupTimer.reset(); warmupTimer.start(); })
            .finallyDo((interrupted) -> {
                warmupTimer.stop();
                launcher.setFrictionWheelVelocity(0);
                launcher.setFeederVelocity(0);
                launcher.setAngleVoltage(0);
                launcher.setTransportVelocity(0);
                intake.setIntakeMotorVelocity(0);
                intake.resetIntakeCounter();
                intake.setSupportMotorVelocity(0);
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
            Commands.parallel(
                Commands.run(() -> {
                    launcher.setFrictionWheelVelocity(feedSpeed);
                    launcher.setFeederVelocity(0);
                })
                .alongWith(launcher.AdjustAngleToPositionCommand(feedAngle))
            ).withTimeout(fastWarmupTime),
            Commands.run(() -> {
                launcher.setFrictionWheelVelocity(feedSpeed);
                launcher.setFeederVelocity(Constants.LauncherConfig.FeederSpeed);
            }, launcher)
        );

        Command transportStream = Commands.sequence(
            Commands.waitSeconds(fastWarmupTime),
            Commands.run(() -> launcher.setTransportVelocity(Constants.TransportConfig.TransportSpeed), launcher)
        );

        Command runIntakeMotors = Commands.run(() -> {
            intake.setIntakeMotorVelocity(Constants.IntakeConfig.IntakeVelocity);
            intake.setSupportMotorVelocity(Constants.IntakeConfig.SupportVelocity);
        });

        Command intakeStream = Commands.sequence(
            Commands.waitSeconds(fastWarmupTime),
            needSwing
                ? Commands.parallel(intake.IntakeFeedingSwingSingleCommand().repeatedly(), runIntakeMotors)
                : runIntakeMotors
        );

        return Commands.parallel(launcherStream, transportStream, intakeStream)
            .finallyDo((interrupted) -> {
                launcher.setFrictionWheelVelocity(0);
                launcher.setFeederVelocity(0);
                launcher.setAngleVoltage(0);
                launcher.setTransportVelocity(0);
                intake.setIntakeMotorVelocity(0);
                intake.setSupportMotorVelocity(0);
                intake.applyIntakePitchMotorNeutral();
            });
    }

    /**
     * Auto dynamic shoot: long warmup to allow pre-positioning before feeding.
     */
    public static Command createAutoDynamicShootingCommand(
        CommandSwerveDrivetrain drive,
        Intake intake,
        Launcher launcher,
        Translation2d blueCenterPosition
    ) {
        double warmupSeconds = Constants.LauncherConfig.AutoDynamicWarmupSeconds;
        Timer warmupTimer = new Timer();

        Command launcherStream = Commands.run(
            () -> {
                boolean isRed = DriverStation.getAlliance()
                    .map(a -> a == Alliance.Red).orElse(false);

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
                    -PITCH_LEAD_RAD_PER_MPS * radialSpeed, -MAX_PITCH_LEAD_RAD, MAX_PITCH_LEAD_RAD);
                double targetPitch = bestPitch - pitchLead;

                launcher.setFrictionWheelVelocity(bestSpeed);
                launcher.setAngleToTarget(targetPitch);
                launcher.setFeederVelocity(
                    warmupTimer.hasElapsed(warmupSeconds) ? Constants.LauncherConfig.FeederSpeed : 0);

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
            Commands.run(() -> launcher.setTransportVelocity(Constants.TransportConfig.TransportSpeed), launcher)
        );

        Command intakeStream = Commands.sequence(
            Commands.waitSeconds(warmupSeconds),
            Commands.runOnce(() -> {
                intake.setIntakeMotorVelocity(Constants.IntakeConfig.IntakeVelocity);
                intake.setSupportMotorVelocity(Constants.IntakeConfig.SupportVelocity);
            }, intake),
            intake.IntakeSwingSingleCommand().repeatedly()
        );

        return Commands.parallel(launcherStream, transportStream, intakeStream)
            .beforeStarting(() -> { warmupTimer.reset(); warmupTimer.start(); })
            .finallyDo((interrupted) -> {
                warmupTimer.stop();
                launcher.setFrictionWheelVelocity(0);
                launcher.setFeederVelocity(0);
                launcher.setAngleVoltage(0);
                launcher.setTransportVelocity(0);
                intake.setIntakeMotorVelocity(0);
                intake.setSupportMotorVelocity(0);
                intake.applyIntakePitchMotorNeutral();
            });
    }
}
