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
import edu.wpi.first.networktables.BooleanEntry;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StringEntry;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ProxyCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.LauncherConfig;
import frc.robot.commands.MagicSequencingCommand;
import frc.robot.commands.MatchStateCommand;
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
import java.util.Set;


public class RobotContainer {

    private final SendableChooser<Command> autoChooser;

    public final Climber climber = new Climber();
    public final Launcher launcher = new Launcher();
    public final Intake intake = new Intake();
    public final CANdleSystem candle = new CANdleSystem();
    public final Vision vision = new Vision();
    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();

    private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond);
    private double MaxAngularRate = RotationsPerSecond.of(1).in(RadiansPerSecond);
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage);
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

    private final Telemetry logger = new Telemetry(MaxSpeed);

    private double launchSpeed = 45.0;
    private double launchAngle = 0.0;

    private final CommandXboxController Driver = new CommandXboxController(0);
    private final CommandXboxController Operator = new CommandXboxController(1);

    // Elastic auto win tracking
    private final BooleanEntry isAutoWinEntry;
    private final StringEntry isAutoWinTextEntry;

    private boolean isSlowMode = false;
    public boolean isVisionPoseFusion = true;

    // Flag to ensure auto-reverse fires at most once per intake cycle
    private boolean autoReverseFired = false;

    public RobotContainer() {

        NetworkTable elasticTable = NetworkTableInstance.getDefault().getTable("Elastic");
        isAutoWinEntry = elasticTable.getBooleanTopic("isAutoWin").getEntry(false);
        isAutoWinTextEntry = elasticTable.getStringTopic("isAutoWinText").getEntry("AUTO LOST");
        isAutoWinEntry.set(false);
        isAutoWinTextEntry.set("AUTO LOST");

        // Start match state tracking when teleop begins
        new Trigger(() -> DriverStation.isTeleopEnabled())
            .onTrue(new MatchStateCommand(() -> isAutoWinEntry.get(false)));

        new EventTrigger("Climb_UP").onTrue(climber.climbingProcessCommand());

        NamedCommands.registerCommand("Climb_DOWN",
            Commands.runOnce(() -> climber.setPosition(ClimbPosition))
        );

        NamedCommands.registerCommand("WarmUp_Auto_Far",
            Commands.run(() -> {
                launcher.setFrictionWheelVelocity(58.5);
                launcher.setAngleToTarget(-0.015);
            }, launcher)
            .until(() -> launcher.isFrictionWheelReady() && launcher.isAngleAtPosition(-0.015))
            .withTimeout(Constants.LauncherConfig.WarmupSecond)
        );

        NamedCommands.registerCommand("WarmUp_Auto_Near",
            Commands.run(() -> {
                launcher.setFrictionWheelVelocity(50);
                launcher.setAngleToTarget(-0.0015);
            }, launcher)
            .until(() -> launcher.isFrictionWheelReady() && launcher.isAngleAtPosition(-0.0015))
            .withTimeout(Constants.LauncherConfig.WarmupSecond)
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
            intake.adjustIntakePositionCommand(IntakeDownPosition)
            .andThen(intake.setIntakeSpeedOneCommand())
            .andThen(intake.intakeCommand())
        );

        NamedCommands.registerCommand("Climb_Auto",
            Commands.sequence(
                climber.climbingProcessCommand()
                    .alongWith(new InstantCommand(() -> candle.changeColor(Constants.RobotState.State.ClimbingUp), candle)),
                Commands.runOnce(() -> climber.setPosition(ClimbPosition), climber)
            )
        );

        configureBindings();

        refreshBackgroundState();

        autoChooser = AutoBuilder.buildAutoChooser();
        SmartDashboard.putData("Auto Chooser", autoChooser);
    }

    public void updateDashboard() {
        SmartDashboard.putNumber("Launcher/Speed", launchSpeed);
        SmartDashboard.putNumber("Launcher/Angle", launchAngle);
        SmartDashboard.putNumber("Launcher/SpeedOffset", Constants.ShootingTrim.speedOffset);
        SmartDashboard.putNumber("Launcher/PitchOffset", Constants.ShootingTrim.pitchOffset);
    }

    public boolean isAutoWin() {
        return isAutoWinEntry.get(false);
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
        Driver.y().onTrue(new InstantCommand(() -> candle.changeColor(Constants.RobotState.State.ClimbingDown), candle));

        Driver.rightTrigger().whileTrue(
            Commands.either(
                Commands.defer(() -> ShootingCommand.createShootingCommand(
                    intake, launcher, launchSpeed, launchAngle)
                    .beforeStarting(() -> candle.changeColor(Constants.RobotState.State.Shooting))
                    .finallyDo(() -> candle.restoreBackground()), 
                    Set.of(intake, launcher)),
                ShootingCommand.createCornerFeedCommand(
                    drivetrain, intake, launcher,
                    () -> -Driver.getLeftY() * MaxSpeed * Constants.DriveConfig.AimDriveScaleX * 0.5,
                    () -> -Driver.getLeftX() * MaxSpeed * Constants.DriveConfig.AimDriveScaleY * 0.5,
                    MaxAngularRate * getInputScale()),
                () -> {
                    boolean isRed = DriverStation.getAlliance()
                        .map(a -> a == DriverStation.Alliance.Red).orElse(false);
                    double x = drivetrain.getPose().getX();
                    double hubX = isRed
                        ? Constants.Layout.FIELD_LENGTH_METERS - Constants.VisionConfig.BLUE_HUB_CENTER.getX()
                        : Constants.VisionConfig.BLUE_HUB_CENTER.getX();
                    return isRed ? x > hubX : x < hubX;
                }
            )
        );

        
        
        Driver.povUp().onTrue(new InstantCommand(() -> launchSpeed += 0.25));
        Driver.povDown().onTrue(new InstantCommand(() -> launchSpeed -= 0.25));
        Driver.povLeft().onTrue(new InstantCommand(() -> launchAngle += 0.0005));
        Driver.povRight().onTrue(new InstantCommand(() -> launchAngle -= 0.0005));

        // Driver.back().whileTrue(launcher.adjustAngleCommand(12));

        // Manual fixed-speed shoot
        // Driver.rightTrigger().whileTrue(
        //     Commands.parallel(
        //         new ProxyCommand(() -> ShootingCommand.createShootingCommand(
        //             intake, launcher,
        //             // LauncherConfig.ManualShootSpeed,
        //             // LauncherConfig.ManualShootAngle
        //             launchSpeed,
        //             launchAngle
        //         )),
        //         Commands.runOnce(() -> candle.changeColor(Constants.RobotState.State.Shooting), candle)
        //     ).finallyDo(() -> candle.restoreBackground())
        // );

        // Move-while-aim dynamic shoot (left trigger)
        Driver.leftTrigger().whileTrue(
            Commands.parallel(
                MoveWhileAimCommand.create(
                    drivetrain,
                    () -> -Driver.getLeftY() * MaxSpeed * Constants.DriveConfig.AimDriveScaleX * 0.25,
                    () -> -Driver.getLeftX() * MaxSpeed * Constants.DriveConfig.AimDriveScaleY * 0.25,
                    MaxAngularRate * getInputScale(),
                    Constants.VisionConfig.BLUE_HUB_CENTER
                ),
                ShootingCommand.createDynamicShootingCommand(
                    drivetrain, intake, launcher,
                    Constants.VisionConfig.BLUE_HUB_CENTER,
                    Constants.LauncherConfig.WarmupSecond
                )
            )
            .beforeStarting(() -> candle.changeColor(Constants.RobotState.State.Shooting))
            .finallyDo(() -> candle.restoreBackground())
        );

        // Intake pitch toggle (up/down)
        Driver.x().onTrue(
            intake.changePitchPositionCommand()
                .andThen(Commands.either(
                    intake.adjustIntakePositionCommand(IntakeUpPosition),
                    intake.adjustIntakePositionCommand(IntakeDownPosition),
                    () -> intake.getIntakePitchFlag()
                ))
        );

        // Intake on/off toggle
        Driver.rightBumper().onTrue(
            intake.changeIntakeSpeedCommand()
            .andThen(intake.intakeCommand())
            .andThen(Commands.either(
                // Intake ON: run transport + feeder + hold friction wheels, reset auto-reverse flag + LED
                Commands.runOnce(() -> {
                    launcher.setTransportVelocity(10);
                    launcher.setFrictionWheelVelocity(-1);
                    launcher.setFeederVelocity(10);
                    launcher.setIntakeBrake(true);
                    autoReverseFired = false;
                }).andThen(new InstantCommand(() -> candle.changeColor(Constants.RobotState.State.Intaking), candle)),
                // Intake OFF: if auto-reverse hasn't fired yet, do 0.3s reverse; otherwise just stop feeder
                Commands.either(
                    Commands.startEnd(
                        () -> { launcher.setFrictionWheelVelocity(-50); launcher.setFeederVelocity(-20); launcher.setTransportVelocity(-10); },
                        () -> { launcher.setFrictionWheelVelocity(0); launcher.setFeederVelocity(0); launcher.setTransportVelocity(0); },
                        launcher
                    ).withTimeout(0.3),
                    Commands.runOnce(() -> launcher.setFeederVelocity(0)),
                    () -> !autoReverseFired
                ).andThen(new InstantCommand(() -> candle.restoreBackground(), candle))
                .andThen(Commands.waitSeconds(0.5))
                .finallyDo(() -> launcher.setIntakeBrake(false)),
                () -> intake.isIntakeRunning()
            ))
        );

        // Auto-reverse: if feeder < 1 rps for 1s while intake is running, trigger reverse once
        new Trigger(() -> !autoReverseFired
                      && intake.isIntakeRunning()
                      && Math.abs(launcher.getFeederVelocity()) < 1.0)
            .debounce(1.0)
            .onTrue(
                Commands.runOnce(() -> autoReverseFired = true)
                .andThen(Commands.startEnd(
                    () -> { launcher.setFrictionWheelVelocity(-50); launcher.setFeederVelocity(-20); launcher.setTransportVelocity(-10); },
                    () -> { launcher.setFrictionWheelVelocity(0); launcher.setFeederVelocity(0); launcher.setTransportVelocity(0); },
                    launcher
                ).withTimeout(0.3))
            );

        // Outtake
        Driver.leftBumper().whileTrue(
            OuttakeCommand.create(intake, launcher, candle)
                .alongWith(new InstantCommand(() -> candle.changeColor(Constants.RobotState.State.Outtaking), candle))
        );

        Driver.back().onTrue(
            launcher.shooterWarmupCommand(Constants.LauncherConfig.WarmupSpeed)
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
                Commands.runOnce(() -> candle.changeColor(Constants.RobotState.State.Shooting), candle)
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
                Commands.runOnce(() -> candle.changeColor(Constants.RobotState.State.Shooting), candle)
            ).finallyDo(() -> candle.restoreBackground())
        );

        // Climber
        Operator.back().onTrue(climber.climbingProcessCommand());
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
                candle.changeColor(Constants.RobotState.State.Shooting);
            })
            .andThen(scoreCmd)
            .finallyDo((interrupted) -> {
                candle.restoreBackground();
                launcher.setFrictionWheelVelocity(0);
            })
            .withTimeout(timeout);

        return stopIntakeAfter ? cmd.andThen(intake.setIntakeSpeedZeroCommand()) : cmd;
    }

    /** Builds a teleop fixed-point auto score command for operator button bindings. */
    private Command makeFixedPointShootCommand(int positionIndex) {
        return Commands.runOnce(() -> {
                isVisionPoseFusion = true;
                candle.changeColor(Constants.RobotState.State.Shooting);
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
