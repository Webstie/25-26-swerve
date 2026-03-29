// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveDrivetrain.SwerveDriveState;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.events.EventTrigger;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ProxyCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.LauncherConfig;
import frc.robot.commands.MagicSequencingCommand;
import frc.robot.commands.MoveWhileAimCommand;
import frc.robot.commands.OuttakeCommand;
import frc.robot.commands.ShootingCommand;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CANdleSystem;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Launcher;
import frc.robot.subsystems.Vision.VisionMeasurement;
import frc.robot.subsystems.Vision;

import static frc.robot.Constants.ClimberConfig.ClimbPosition;
import static frc.robot.Constants.IntakeConfig.*;
import java.util.List;


public class RobotContainer {

    private final SendableChooser<Command> autoChooser;

    public final Climber climber = new Climber();
    public final Launcher launcher = new Launcher();
    public final Intake intake = new Intake();
    public final CANdleSystem candle = new CANdleSystem();
    public final Vision vision = new Vision();
    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();

    private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond);
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond);
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage);
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

    private final Telemetry logger = new Telemetry(MaxSpeed);

    private final CommandXboxController Driver = new CommandXboxController(0);
    private final CommandXboxController Operator = new CommandXboxController(1);

    private boolean isSlowMode = false;
    public boolean isVisionPoseFusion = true;

    public RobotContainer() {

        new EventTrigger("Climb_UP").onTrue(climber.ClimbingProcessSingleCommand());

        NamedCommands.registerCommand("Climb_DOWN",
            Commands.runOnce(() -> climber.setPosition(ClimbPosition))
        );

        NamedCommands.registerCommand("WarmUp_Auto_Far",
            Commands.parallel(
                Commands.runOnce(() -> launcher.setFrictionWheelVelocity(58.5)),
                launcher.AdjustAngleToPositionCommand(-0.015)
            )
        );

        NamedCommands.registerCommand("WarmUp_Auto_Near",
            Commands.parallel(
                Commands.runOnce(() -> launcher.setFrictionWheelVelocity(50)),
                launcher.AdjustAngleToPositionCommand(-0.0015)
            )
        );

        // Auto shoot commands: positionIndex, timeout, stopIntakeAfter, useFast
        NamedCommands.registerCommand("Shoot_Auto_Blue_Far_Left",         makeAutoScoreCommand(3, 6.0,  false, true));
        NamedCommands.registerCommand("Shoot_Auto_Blue_Far_Left_toEnd",   makeAutoScoreCommand(3, 10.0, true,  true));
        NamedCommands.registerCommand("Shoot_Auto_Blue_Far_Right",        makeAutoScoreCommand(5, 5.0,  false, true));
        NamedCommands.registerCommand("Shoot_Auto_Blue_Near_Mid",         makeAutoScoreCommand(1, 3.0,  true,  true));
        NamedCommands.registerCommand("Shoot_Auto_Blue_Near_Right",       makeAutoScoreCommand(2, 5.0,  true,  true));
        NamedCommands.registerCommand("Shoot_Auto_Blue_Near_Right_toEnd", makeAutoScoreCommand(2, 10.0, true,  true));
        NamedCommands.registerCommand("Shoot_Auto_Blue_Near_Left",        makeAutoScoreCommand(0, 10.0, true,  true));
        NamedCommands.registerCommand("Shoot_Auto_Fixed_Blue_Near_Mid",   makeAutoScoreCommand(1, 5.0,  false, false));

        NamedCommands.registerCommand("Intake_Auto",
            intake.AdjustIntakePositionSingleCommand(IntakeDownPosition)
            .andThen(intake.SetIntakeSpeedOneSingleCommand())
            .andThen(intake.IntakeSingleCommand())
        );

        NamedCommands.registerCommand("Climb_Auto",
            Commands.sequence(
                climber.ClimbingProcessSingleCommand()
                    .alongWith(new InstantCommand(() -> candle.Changecolor(Constants.RobotState.State.ClimbingUp), candle)),
                Commands.runOnce(() -> climber.setPosition(ClimbPosition), climber)
            )
        );

        configureBindings();

        refreshBackgroundState();

        autoChooser = AutoBuilder.buildAutoChooser();
        SmartDashboard.putData("Auto Chooser", autoChooser);
    }

    public void updateDashboard() {
        SmartDashboard.putNumber("Launcher/SpeedOffset", Constants.ShootingTrim.speedOffset);
        SmartDashboard.putNumber("Launcher/PitchOffset", Constants.ShootingTrim.pitchOffset);
    }

    private void configureBindings() {

        /*** Driver ***/
        drivetrain.setDefaultCommand(
            drivetrain.applyRequest(() -> {
                double rawX = MathUtil.applyDeadband(-Driver.getLeftY(), 0.1);
                double rawY = MathUtil.applyDeadband(-Driver.getLeftX(), 0.1);
                double rawR = MathUtil.applyDeadband(-Driver.getRightX(), 0.1);
                double scale = getInputScale();
                return drive
                    .withVelocityX(rawX * MaxSpeed * Constants.DriveConfig.TeleopDriveSpeedScale * scale)
                    .withVelocityY(rawY * MaxSpeed * Constants.DriveConfig.TeleopDriveSpeedScale * scale)
                    .withRotationalRate(rawR * MaxAngularRate * scale);
            })
        );
        drivetrain.registerTelemetry(logger::telemeterize);

        // Reset field-centric heading
        Driver.a().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));

        // Toggle vision pose fusion
        Driver.b().onTrue(Commands.runOnce(() -> {
            isVisionPoseFusion = !isVisionPoseFusion;
            refreshBackgroundState();
        }));

        // Toggle slow mode (25% speed cap)
        Driver.start().onTrue(Commands.runOnce(() -> {
            isSlowMode = !isSlowMode;
            refreshBackgroundState();
        }));

        // Celebration lights
        Driver.y().onTrue(new InstantCommand(() -> candle.Changecolor(Constants.RobotState.State.ClimbingDown), candle));

        // Manual fixed-speed shoot
        Driver.rightTrigger().whileTrue(
            Commands.parallel(
                new ProxyCommand(() -> ShootingCommand.createShootingCommand(
                    intake, launcher,
                    LauncherConfig.ManualShootSpeed,
                    LauncherConfig.ManualShootAngle
                )),
                Commands.runOnce(() -> candle.Changecolor(Constants.RobotState.State.Shooting), candle)
            ).finallyDo(() -> candle.restoreBackground())
        );

        // Move-while-aim dynamic shoot (left trigger)
        Driver.leftTrigger().whileTrue(
            Commands.parallel(
                MoveWhileAimCommand.create(
                    drivetrain,
                    () -> -Driver.getLeftY() * MaxSpeed * Constants.DriveConfig.AimDriveScaleX * getInputScale(),
                    () -> -Driver.getLeftX() * MaxSpeed * Constants.DriveConfig.AimDriveScaleY * getInputScale(),
                    MaxAngularRate * getInputScale(),
                    Constants.VisionConfig.BLUE_HUB_CENTER
                ),
                ShootingCommand.createDynamicShootingCommand(
                    drivetrain, intake, launcher,
                    Constants.VisionConfig.BLUE_HUB_CENTER
                )
            )
            .beforeStarting(() -> candle.Changecolor(Constants.RobotState.State.Shooting))
            .finallyDo(() -> candle.restoreBackground())
        );

        // Intake pitch toggle (up/down)
        Driver.x().onTrue(
            intake.ChangePitchPositionSingleCommand()
                .andThen(Commands.either(
                    intake.AdjustIntakePositionSingleCommand(IntakeUpPosition),
                    intake.AdjustIntakePositionSingleCommand(IntakeDownPosition),
                    () -> intake.getIntakePitchFlag()
                ))
        );

        // Intake on/off toggle
        Driver.rightBumper().onTrue(
            intake.ChangeIntakeSpeedSingleCommand()
            .andThen(intake.IntakeSingleCommand())
            .andThen(Commands.either(
                new InstantCommand(() -> candle.Changecolor(Constants.RobotState.State.Intaking), candle),
                new InstantCommand(() -> candle.restoreBackground(), candle),
                () -> intake.isIntakeRunning()
            ))
        );

        // Outtake
        Driver.leftBumper().whileTrue(
            OuttakeCommand.create(intake, launcher, candle)
                .alongWith(new InstantCommand(() -> candle.Changecolor(Constants.RobotState.State.Outtaking), candle))
        );

        /*** Operator ***/

        // Mid-field feed with intake swing
        Operator.a().whileTrue(
            Commands.parallel(
                ShootingCommand.createDynamicFeedCommand(
                    intake, launcher,
                    LauncherConfig.MidFieldFeedSpeed,
                    LauncherConfig.MidFieldFeedAngle,
                    true
                ),
                Commands.runOnce(() -> candle.Changecolor(Constants.RobotState.State.Shooting), candle)
            ).finallyDo(() -> candle.restoreBackground())
        );

        // Mid-field feed without intake swing
        Operator.povDown().whileTrue(
            Commands.parallel(
                ShootingCommand.createDynamicFeedCommand(
                    intake, launcher,
                    LauncherConfig.MidFieldFeedSpeed,
                    LauncherConfig.MidFieldFeedAngle,
                    false
                ),
                Commands.runOnce(() -> candle.Changecolor(Constants.RobotState.State.Shooting), candle)
            ).finallyDo(() -> candle.restoreBackground())
        );

        // Climber
        Operator.back().onTrue(climber.ClimbingProcessSingleCommand());
        Operator.start().whileTrue(
            Commands.run(() -> climber.setPosition(ClimbPosition))
        ).onFalse(
            Commands.sequence(
                Commands.runOnce(() -> climber.releaseClimber()),
                Commands.waitSeconds(0.1),
                Commands.runOnce(() -> climber.setPosition(climber.getCurrentPosition()))
            )
        );

        // Fixed-point auto score: near left/mid/right (0-2), far left/mid/right (3-5)
        Operator.x().whileTrue(makeFixedPointShootCommand(0));        // near left
        Operator.y().whileTrue(makeFixedPointShootCommand(1));        // near mid
        Operator.b().whileTrue(makeFixedPointShootCommand(2));        // near right
        Operator.povLeft().whileTrue(makeFixedPointShootCommand(3));  // far left
        Operator.povUp().whileTrue(makeFixedPointShootCommand(4));    // far mid
        Operator.povRight().whileTrue(makeFixedPointShootCommand(5)); // far right

        // Global shooting trim: rightBumper/rightTrigger = speed ±step, leftBumper/leftTrigger = pitch ±step
        Operator.rightBumper().onTrue(Commands.runOnce(() ->
            Constants.ShootingTrim.speedOffset += Constants.ShootingTrim.SPEED_TRIM_STEP));
        Operator.rightTrigger().onTrue(Commands.runOnce(() ->
            Constants.ShootingTrim.speedOffset -= Constants.ShootingTrim.SPEED_TRIM_STEP));
        Operator.leftBumper().onTrue(Commands.runOnce(() ->
            Constants.ShootingTrim.pitchOffset += Constants.ShootingTrim.PITCH_TRIM_STEP));
        Operator.leftTrigger().onTrue(Commands.runOnce(() ->
            Constants.ShootingTrim.pitchOffset -= Constants.ShootingTrim.PITCH_TRIM_STEP));
    }

    public void addMeasurements() {
        SwerveDriveState driveState = drivetrain.getState();
        List<VisionMeasurement> measurements = vision.processVisionData(driveState);
        for (VisionMeasurement m : measurements) {
            drivetrain.addVisionMeasurement(m.pose, m.timestamp, m.stdDevs);
        }
    }

    public Command getAutoInitCommand() {
        return AutoBuilder.resetOdom(Constants.VisionConfig.m_initialPose);
    }

    public Command getAutonomousCommand() {
        return autoChooser.getSelected();
    }

    private double getInputScale() {
        return isSlowMode ? 0.25 : 1.0;
    }

    /**
     * Refreshes the background LED state based on persistent mode flags.
     * Priority: SlowMode (cyan) > VisionFusion disabled (yellow) > Idle (off).
     */
    private void refreshBackgroundState() {
        if (isSlowMode) {
            candle.setBackgroundState(Constants.RobotState.State.ClimbingUp);
        } else if (!isVisionPoseFusion) {
            candle.setBackgroundState(Constants.RobotState.State.VisionFusion);
        } else {
            candle.setBackgroundState(Constants.RobotState.State.Idle);
        }
    }

    /** Builds a named-command auto score sequence for a fixed field position. */
    private Command makeAutoScoreCommand(int positionIndex, double timeout,
                                          boolean stopIntakeAfter, boolean useFast) {
        Command scoreCmd = useFast
            ? MagicSequencingCommand.createFastFixedPointAutoScoreCommand(
                positionIndex, drivetrain, intake, launcher,
                Constants.VisionConfig.BLUE_HUB_CENTER,
                Constants.VisionConfig.POINTS_PARAMS_TABLE_BLUE)
            : MagicSequencingCommand.createFixedPointAutoScoreCommand(
                positionIndex, drivetrain, intake, launcher,
                Constants.VisionConfig.BLUE_HUB_CENTER,
                Constants.VisionConfig.POINTS_PARAMS_TABLE_BLUE);

        Command cmd = Commands.runOnce(() -> {
                isVisionPoseFusion = true;
                candle.Changecolor(Constants.RobotState.State.Shooting);
            })
            .andThen(scoreCmd)
            .finallyDo((interrupted) -> {
                candle.restoreBackground();
                launcher.setFrictionWheelVelocity(0);
            })
            .withTimeout(timeout);

        return stopIntakeAfter ? cmd.andThen(intake.SetIntakeSpeedZeroSingleCommand()) : cmd;
    }

    /** Builds a teleop fixed-point auto score command for operator button bindings. */
    private Command makeFixedPointShootCommand(int positionIndex) {
        return Commands.runOnce(() -> {
                isVisionPoseFusion = true;
                candle.Changecolor(Constants.RobotState.State.Shooting);
            })
            .andThen(MagicSequencingCommand.createFixedPointAutoScoreCommand(
                positionIndex, drivetrain, intake, launcher,
                Constants.VisionConfig.BLUE_HUB_CENTER,
                Constants.VisionConfig.POINTS_PARAMS_TABLE_BLUE))
            .finallyDo((interrupted) -> {
                candle.restoreBackground();
                launcher.setFrictionWheelVelocity(0);
            });
    }
}
