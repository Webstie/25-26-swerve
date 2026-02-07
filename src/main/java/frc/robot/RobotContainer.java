// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.commands.ShootingCommand;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.TransportSubsystem;
import static frc.robot.Constants.Intake.*;


public class RobotContainer {
    private final SendableChooser<Command> autoChooser;
    public final ClimberSubsystem Climber = new ClimberSubsystem();
    public final ShooterSubsystem Shooter = new ShooterSubsystem();
    public final TransportSubsystem Transport = new TransportSubsystem();
    public final IntakeSubsystem Intake = new IntakeSubsystem();

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
        configureBindings();
        // Build an auto chooser. This will use Commands.none() as the default option.
        autoChooser = AutoBuilder.buildAutoChooser();

            // Another option that allows you to specify the default auto by its name
            // autoChooser = AutoBuilder.buildAutoChooser("My Default Auto");

            SmartDashboard.putData("Auto Chooser", autoChooser);
        }

    private void configureBindings() {
        
        //Driver
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

        // Run SysId routines when holding back/start and X/Y.
        // Note that each routine should be run exactly once in a single log.
        Driver.back().and(Driver.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
        Driver.back().and(Driver.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
        Driver.start().and(Driver.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
        Driver.start().and(Driver.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

        // reset the field-centric heading on left bumper press
        Driver.leftBumper().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));

        drivetrain.registerTelemetry(logger::telemeterize);

        

        //Operator
        // Operator.a().onTrue(Intake.Intake_up_presstimes().andThen(Intake.IntakeCommand()));
        Operator.x().onTrue(Intake.changePositionFlag()
                    .andThen(Commands.either(
                        Intake.adjust_IntakePosition(IntakeUpPosition).andThen(Intake.IntakeCommand()), 
                        Intake.adjust_IntakePosition(IntakeDownPosition).andThen(Intake.IntakeCommand()), 
                        ()->Intake.IntakepitchPositionFlag)));
        Operator.y().onTrue(Intake.OuttakeCommand());
        
        Operator.leftBumper().whileTrue(new ShootingCommand(Intake, Shooter, Transport));

        Operator.rightBumper().onTrue(Climber.StartClimb());
        Operator.rightTrigger().onTrue(Climber.Climb());




    }
    
   public Command getAutonomousCommand() {
    // This method loads the auto when it is called, however, it is recommended
    // to first load your paths/autos when code starts, then return the
    // pre-loaded auto/path
    return new PathPlannerAuto("New Auto");
  }
}
