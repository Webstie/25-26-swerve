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
import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.MagicSequencingCommand;
import frc.robot.commands.OuttakeCommand;
import frc.robot.commands.ShootingCommand;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CANdleSystem;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Launcher;
import frc.robot.subsystems.Transport;
import frc.robot.subsystems.Vision.VisionMeasurement;
import frc.robot.subsystems.Vision;

import static frc.robot.Constants.IntakeConfig.*;

import java.util.List;
import java.util.Set;

import static frc.robot.Constants.LauncherConfig.shootingVoltage;


public class RobotContainer {
    private final SendableChooser<Command> autoChooser;
    public final Climber climber = new Climber();
    public final Launcher launcher = new Launcher();
    public final Transport transport = new Transport();
    public final Intake intake = new Intake();
    public final CANdleSystem candle = new CANdleSystem();
    public final Vision vision = new Vision();

    private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity

    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

    private final Telemetry logger = new Telemetry(MaxSpeed);
    
    private final CommandXboxController Driver = new CommandXboxController(0);
    private final CommandXboxController Operator = new CommandXboxController(1);

    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();

    public RobotContainer() {

        //register the named commands for auto mode
        //自动发射调用命令
        NamedCommands.registerCommand("ShootNamedCommand",
            ShootingCommand.createShootingCommand(intake, launcher, transport).withTimeout(5.0)
        );

        //自动intake调用命令
        NamedCommands.registerCommand("IntakeNamedCommand",
            intake.AdjustIntakePositionSingleCommand(IntakeDownPosition)
            .andThen(intake.ChangeIntakeSpeedSingleCommand())
            .andThen(intake.IntakeSingleCommand())
            .andThen(Commands.either(
                new InstantCommand(() -> candle.Changecolor(Constants.RobotState.State.STATE1), candle),
                new InstantCommand(() -> candle.Changecolor(Constants.RobotState.State.STATE2), candle),
                () -> intake.Intake_press_times % 2 == 0
            ))
        );

        NamedCommands.registerCommand("ClimbingNamedCommand", 
            intake.AdjustIntakePositionSingleCommand(IntakeUpPosition)
                        .andThen(intake.ChangeIntakeSpeedSingleCommand())
            .andThen(intake.IntakeSingleCommand())
            .andThen(Commands.either(
                new InstantCommand(() -> candle.Changecolor(Constants.RobotState.State.STATE1), candle),
                new InstantCommand(() -> candle.Changecolor(Constants.RobotState.State.STATE2), candle),
                () -> intake.Intake_press_times % 2 == 0
            ))
            
        );


        configureBindings();
        // Build an auto chooser. This will use Commands.none() as the default option.
        autoChooser = AutoBuilder.buildAutoChooser();

            // Another option that allows you to specify the default auto by its name
            // autoChooser = AutoBuilder.buildAutoChooser("My Default Auto");

            SmartDashboard.putData("Auto Chooser", autoChooser);
        }

    private void configureBindings() {
        
        /******************************************************Driver**********************************************************************/
        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.
        drivetrain.setDefaultCommand(
            // Drivetrain will execute this command periodically
            drivetrain.applyRequest(() ->
                drive.withVelocityX(-Driver.getLeftY() * MaxSpeed * 0.5) // Drive forward with negative Y (forward)
                    .withVelocityY(-Driver.getLeftX() * MaxSpeed * 0.5) // Drive left with negative X (left)
                    .withRotationalRate(-Driver.getRightX() * MaxAngularRate) // Drive counterclockwise with negative X (left)
            )
        );

        Driver.a().whileTrue(drivetrain.applyRequest(() -> brake));
        Driver.b().whileTrue(drivetrain.applyRequest(() ->
            point.withModuleDirection(new Rotation2d(-Driver.getLeftY(), -Driver.getLeftX()))
        ));

        // reset the field-centric heading on left bumper press
        Driver.leftBumper().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));

        drivetrain.registerTelemetry(logger::telemeterize);

        Driver.y().whileTrue(
            Commands.runOnce(() -> {
                System.out.println("Starting Hub targeting command");
            })
            .andThen(MagicSequencingCommand.createSequentialAutoScoreCommand(
                0,
                drivetrain, 
                intake, 
                launcher, 
                transport,
                Constants.VisionConfig.BLUE_SCORING_NODES, 
                Constants.VisionConfig.BLUE_HUB_CENTER
            ))
            .finallyDo((interrupted) -> {
                System.out.println("Hub targeting command ended. Interrupted: " + interrupted);
            })
        );

        Driver.b().whileTrue(
            Commands.runOnce(() -> {
                System.out.println("Starting Hub targeting command");
            })
            .andThen(MagicSequencingCommand.createSequentialAutoScoreCommand(
                1,
                drivetrain, 
                intake, 
                launcher, 
                transport,
                Constants.VisionConfig.BLUE_SCORING_NODES, 
                Constants.VisionConfig.BLUE_HUB_CENTER
            ))
            .finallyDo((interrupted) -> {
                System.out.println("Hub targeting command ended. Interrupted: " + interrupted);
            })
        );

        /**********************************************************Operator**********************************************************/
        // Operator.a().onTrue(Intake.Intake_up_presstimes().andThen(Intake.IntakeCommand()));
        Operator.x().onTrue(intake.ChangePitchPositionSingleCommand()
                    .andThen(Commands.either(
                        intake.AdjustIntakePositionSingleCommand(IntakeUpPosition), 
                        intake.AdjustIntakePositionSingleCommand(IntakeDownPosition), 
                        ()->intake.IntakepitchPositionFlag)));
        Operator.y().onTrue(
            intake.ChangeIntakeSpeedSingleCommand()
                .andThen(intake.IntakeSingleCommand())
                .andThen(Commands.either(
                    new InstantCommand(() -> candle.Changecolor(Constants.RobotState.State.STATE1), candle),
                    new InstantCommand(() -> candle.Changecolor(Constants.RobotState.State.STATE2), candle),
                    () -> intake.Intake_press_times % 2 == 0
                ))
        );
        

        Operator.a().whileTrue(
            new OuttakeCommand(intake, launcher, transport)
        );

        Operator.povRight().whileTrue(
            Commands.runOnce(() -> {
                System.out.println("SWING");
            })
            .andThen(
                Commands.runOnce(()->
                    intake.IntakeSwingSingleCommand().repeatedly())
            ));


        //test candle
        Operator.b().onTrue((new InstantCommand(() -> candle.Changecolor(Constants.RobotState.State.STATE4), candle)));

        Operator.leftBumper().whileTrue(
            ShootingCommand.createShootingCommand(intake, launcher, transport)
        );

        Operator.rightBumper().onTrue(climber.ClimbingProcessSingleCommand());
        Operator.rightTrigger().onTrue(climber.ClimbSingleCommand());

        Operator.povUp().onTrue(launcher.AdjustAngleToPositionCommand(0.02));
        Operator.povDown().onTrue(launcher.AdjustAngleToPositionCommand(0));


        //*****************************************************sysid ********************************************************************************/
        // // Run SysId routines when holding back/start and X/Y.
        // // Note that each routine should be run exactly once in a single log.
        // Driver.back().and(Driver.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
        // Driver.back().and(Driver.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
        // Driver.start().and(Driver.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
        // Driver.start().and(Driver.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));


    }
    
    public void addMeasurements() {

        SwerveDriveState driveState = drivetrain.getState();
        List<VisionMeasurement> measurements = vision.processVisionData(driveState);

        for (VisionMeasurement m : measurements) {
            drivetrain.addVisionMeasurement(m.pose, m.timestamp, m.stdDevs);
        }

    }

    public Command getAutoInitCommand(){
        return AutoBuilder.resetOdom(Constants.VisionConfig.m_initialPose);
    }

    public Command getAutonomousCommand() {
        // This method loads the auto when it is called, however, it is recommended
        // to first load your paths/autos when code starts, then return the
        // pre-loaded auto/path
        return autoChooser.getSelected();
    }
}
