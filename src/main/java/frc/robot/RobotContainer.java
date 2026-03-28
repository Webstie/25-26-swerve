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
import com.pathplanner.lib.events.EventTrigger;
import com.pathplanner.lib.events.TriggerEvent;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ProxyCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.LauncherConfig;
import frc.robot.commands.AutoMoveWhileAimCommand;
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
import frc.robot.subsystems.Transport;
import frc.robot.subsystems.Vision.VisionMeasurement;
import frc.robot.subsystems.Vision;

import static frc.robot.Constants.ClimberConfig.ClimbPosition;
import static frc.robot.Constants.IntakeConfig.*;
import java.util.List;
import java.util.Locale.LanguageRange;


public class RobotContainer {
     
    //auto
    private final SendableChooser<Command> autoChooser;

    //subsystems
    public final Climber climber = new Climber();
    public final Launcher launcher = new Launcher();
    public final Transport transport = new Transport();
    public final Intake intake = new Intake();
    public final CANdleSystem candle = new CANdleSystem();
    public final Vision vision = new Vision();
    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();

    //底盘参数设置
    private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity
    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // deadband applied manually in lambda
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

    private final Telemetry logger = new Telemetry(MaxSpeed);
    
    //joystick
    private final CommandXboxController Driver = new CommandXboxController(0);
    private final CommandXboxController Operator = new CommandXboxController(1);

    //是否开启半速模式
    private boolean isSlowMode = false;

    //调试的发射参数，目前没有实际用途
    private double launchSpeed = 50;
    private double launchAngle = -0.01;

    //是否融合视觉位姿
    public boolean isVisionPoseFusion = true;

    public RobotContainer() {

        new EventTrigger("Climb_UP").onTrue(climber.ClimbingProcessSingleCommand());

        //register the named commands for auto mode
        //爬升到位命令，直接设定目标位置
        NamedCommands.registerCommand("Climb_DOWN",
            Commands.runOnce(() -> {
                climber.setPosition(ClimbPosition);
            })
        );
        
        //远预热命令，设置摩擦轮速度并调整角度到合适范围
        NamedCommands.registerCommand("WarmUp_Auto_Far",
            Commands.parallel(
                Commands.runOnce(()->launcher.setFrictionWheelVelocity(58.5)),//预热       
                launcher.AdjustAngleToPositionCommand(-0.015)// 调整角度
            )
        );

        //近预热命令，设置摩擦轮速度并调整角度到合适范围
        NamedCommands.registerCommand("WarmUp_Auto_Near",
            Commands.parallel(
                Commands.runOnce(()->launcher.setFrictionWheelVelocity(50)),//预热       
                launcher.AdjustAngleToPositionCommand(-0.0015)// 调整角度
            )
        );

        //自动发射远左
        NamedCommands.registerCommand("Shoot_Auto_Blue_Far_Left",
            Commands.runOnce(() -> {
                System.out.println("Starting Hub targeting command");
                isVisionPoseFusion = true;
                candle.Changecolor(Constants.RobotState.State.Shooting);
            })
            .andThen(MagicSequencingCommand.createFastFixedPointAutoScoreCommand(
                3,
                drivetrain, 
                intake, 
                launcher, 
                transport,
                Constants.VisionConfig.BLUE_HUB_CENTER,
                Constants.VisionConfig.POINTS_PARAMS_TABLE_BLUE
            ))
            .finallyDo((interrupted) -> {
                candle.restoreBackground();
                launcher.setFrictionWheelVelocity(0);//防止半自动预热后被中断导致摩擦轮一直转
                System.out.println("Hub targeting command ended. Interrupted: " + interrupted);
            }).withTimeout(6.0)
            // .andThen(intake.SetIntakeSpeedZeroSingleCommand())
        );

        //自动发射远左--不断发射直到结束
        NamedCommands.registerCommand("Shoot_Auto_Blue_Far_Left_toEnd",
            Commands.runOnce(() -> {
                System.out.println("Starting Hub targeting command");
                isVisionPoseFusion = true;
                candle.Changecolor(Constants.RobotState.State.Shooting);
            })
            .andThen(MagicSequencingCommand.createFastFixedPointAutoScoreCommand(
                3,
                drivetrain, 
                intake, 
                launcher, 
                transport,
                Constants.VisionConfig.BLUE_HUB_CENTER,
                Constants.VisionConfig.POINTS_PARAMS_TABLE_BLUE
            ))
            .finallyDo((interrupted) -> {
                candle.restoreBackground();
                launcher.setFrictionWheelVelocity(0);//防止半自动预热后被中断导致摩擦轮一直转
                System.out.println("Hub targeting command ended. Interrupted: " + interrupted);
            }).withTimeout(10.0)
            .andThen(intake.SetIntakeSpeedZeroSingleCommand())
        );

        //自动发射远右
        NamedCommands.registerCommand("Shoot_Auto_Blue_Far_Right",
            Commands.runOnce(() -> {
                System.out.println("Starting Hub targeting command");
                isVisionPoseFusion = true;
                candle.Changecolor(Constants.RobotState.State.Shooting);
            })
            .andThen(MagicSequencingCommand.createFastFixedPointAutoScoreCommand(
                5,
                drivetrain, 
                intake, 
                launcher, 
                transport,
                Constants.VisionConfig.BLUE_HUB_CENTER,
                Constants.VisionConfig.POINTS_PARAMS_TABLE_BLUE
            ))
            .finallyDo((interrupted) -> {
                candle.restoreBackground();
                launcher.setFrictionWheelVelocity(0);//防止半自动预热后被中断导致摩擦轮一直转
                System.out.println("Hub targeting command ended. Interrupted: " + interrupted);
            }).withTimeout(5.0)
            // .andThen(intake.SetIntakeSpeedZeroSingleCommand())
        );
        
        //自动发射近中
        NamedCommands.registerCommand("Shoot_Auto_Blue_Near_Mid",
            Commands.runOnce(() -> {
                System.out.println("Starting Hub targeting command");
                isVisionPoseFusion = true;
                candle.Changecolor(Constants.RobotState.State.Shooting);
            })
            .andThen(MagicSequencingCommand.createFastFixedPointAutoScoreCommand(
                1,
                drivetrain, 
                intake, 
                launcher, 
                transport,
                Constants.VisionConfig.BLUE_HUB_CENTER,
                Constants.VisionConfig.POINTS_PARAMS_TABLE_BLUE
            ))
            .finallyDo((interrupted) -> {
                candle.restoreBackground();
                launcher.setFrictionWheelVelocity(0);//防止半自动预热后被中断导致摩擦轮一直转
                System.out.println("Hub targeting command ended. Interrupted: " + interrupted);
            }).withTimeout(3.0)
            .andThen(intake.SetIntakeSpeedZeroSingleCommand())
        );

        //自动发射近右
        NamedCommands.registerCommand("Shoot_Auto_Blue_Near_Right",
            Commands.runOnce(() -> {
                System.out.println("Starting Hub targeting command");
                isVisionPoseFusion = true;
                candle.Changecolor(Constants.RobotState.State.Shooting);
            })
            .andThen(MagicSequencingCommand.createFastFixedPointAutoScoreCommand(
                2,
                drivetrain, 
                intake, 
                launcher, 
                transport,
                Constants.VisionConfig.BLUE_HUB_CENTER,
                Constants.VisionConfig.POINTS_PARAMS_TABLE_BLUE
            ))
            .finallyDo((interrupted) -> {
                candle.restoreBackground();
                launcher.setFrictionWheelVelocity(0);//防止半自动预热后被中断导致摩擦轮一直转
                System.out.println("Hub targeting command ended. Interrupted: " + interrupted);
            }).withTimeout(5.0)
            .andThen(intake.SetIntakeSpeedZeroSingleCommand())
        );

        //自动发射近右到最后
        NamedCommands.registerCommand("Shoot_Auto_Blue_Near_Right_toEnd",
            Commands.runOnce(() -> {
                System.out.println("Starting Hub targeting command");
                isVisionPoseFusion = true;
                candle.Changecolor(Constants.RobotState.State.Shooting);
            })
            .andThen(MagicSequencingCommand.createFastFixedPointAutoScoreCommand(
                2,
                drivetrain, 
                intake, 
                launcher, 
                transport,
                Constants.VisionConfig.BLUE_HUB_CENTER,
                Constants.VisionConfig.POINTS_PARAMS_TABLE_BLUE
            ))
            .finallyDo((interrupted) -> {
                candle.restoreBackground();
                launcher.setFrictionWheelVelocity(0);//防止半自动预热后被中断导致摩擦轮一直转
                System.out.println("Hub targeting command ended. Interrupted: " + interrupted);
            }).withTimeout(10.0)
            .andThen(intake.SetIntakeSpeedZeroSingleCommand())
        );

        //自动发射近左
        NamedCommands.registerCommand("Shoot_Auto_Blue_Near_Left",
            Commands.runOnce(() -> {
                System.out.println("Starting Hub targeting command");
                isVisionPoseFusion = true;
                candle.Changecolor(Constants.RobotState.State.Shooting);
            })
            .andThen(MagicSequencingCommand.createFastFixedPointAutoScoreCommand(
                0,
                drivetrain, 
                intake, 
                launcher, 
                transport,
                Constants.VisionConfig.BLUE_HUB_CENTER,
                Constants.VisionConfig.POINTS_PARAMS_TABLE_BLUE
            ))
            .finallyDo((interrupted) -> {
                candle.restoreBackground();
                launcher.setFrictionWheelVelocity(0);//防止半自动预热后被中断导致摩擦轮一直转
                System.out.println("Hub targeting command ended. Interrupted: " + interrupted);
            }).withTimeout(10.0)
            .andThen(intake.SetIntakeSpeedZeroSingleCommand())
        );

        //自动发射远左，求稳移动到固定点位
        NamedCommands.registerCommand("Shoot_Auto_Fixed_Blue_Near_Mid",
            Commands.runOnce(() -> {
                System.out.println("Starting Hub targeting command");
                isVisionPoseFusion = true;
                candle.Changecolor(Constants.RobotState.State.Shooting);
            })
            .andThen(MagicSequencingCommand.createFixedPointAutoScoreCommand(
                1,
                drivetrain, 
                intake, 
                launcher, 
                transport,
                Constants.VisionConfig.BLUE_HUB_CENTER,
                Constants.VisionConfig.POINTS_PARAMS_TABLE_BLUE
            ))
            .finallyDo((interrupted) -> {
                //isVisionPoseFusion = false; // 退出半自动模式，关闭视觉位姿融合
                candle.restoreBackground();
                launcher.setFrictionWheelVelocity(0);//防止半自动预热后被中断导致摩擦轮一直转
                System.out.println("Hub targeting command ended. Interrupted: " + interrupted);
            }).withTimeout(5.0)
            // .andThen(intake.SetIntakeSpeedZeroSingleCommand())
        );

        //********************************（deprecated）*******************************
        // //自动发射远右，求稳移动到固定点位
        // NamedCommands.registerCommand("Shoot_Auto_Fixed_Blue_Far_Right",
        //     Commands.runOnce(() -> {
        //         System.out.println("Starting Hub targeting command");
        //         isVisionPoseFusion = true;
        //         candle.Changecolor(Constants.RobotState.State.Shooting);
        //     })
        //     .andThen(MagicSequencingCommand.createFixedPointAutoScoreCommand(
        //         5,
        //         drivetrain, 
        //         intake, 
        //         launcher, 
        //         transport,
        //         Constants.VisionConfig.BLUE_HUB_CENTER,
        //         Constants.VisionConfig.POINTS_PARAMS_TABLE_BLUE
        //     ))
        //     .finallyDo((interrupted) -> {
        //         //isVisionPoseFusion = false; // 退出半自动模式，关闭视觉位姿融合
        //         candle.restoreBackground();
        //         launcher.setFrictionWheelVelocity(0);//防止半自动预热后被中断导致摩擦轮一直转
        //         System.out.println("Hub targeting command ended. Interrupted: " + interrupted);
        //     }).withTimeout(5.0)
        //     // .andThen(intake.SetIntakeSpeedZeroSingleCommand())
        // );
        
        //********************************（deprecated）*******************************
        // //自动发射跑打
        // NamedCommands.registerCommand("Shoot_Auto_Dynamic",
        //     Commands.parallel(
        //             AutoMoveWhileAimCommand.create(  // <--- 修改这里
        //                 drivetrain,
        //                 Constants.VisionConfig.BLUE_HUB_CENTER
        //             ),
        //             ShootingCommand.createAutoDynamicShootingCommand(
        //                 drivetrain,
        //                 intake,
        //                 launcher,
        //                 transport,
        //                 Constants.VisionConfig.BLUE_HUB_CENTER
        //             )
        //         )
        //     .beforeStarting(() -> candle.Changecolor(Constants.RobotState.State.Shooting))
        //     .finallyDo(() -> candle.restoreBackground())
        //     .withTimeout(5.0) // 运行4秒后，瞄准逻辑会自动卸载，恢复正常寻路
        //     // .andThen(intake.SetIntakeSpeedZeroSingleCommand())
        // );

        //自动intake调用命令
        NamedCommands.registerCommand("Intake_Auto",
            intake.AdjustIntakePositionSingleCommand(IntakeDownPosition)
            .andThen(intake.SetIntakeSpeedOneSingleCommand())
            .andThen(intake.IntakeSingleCommand())
        );

        
        NamedCommands.registerCommand("Climb_Auto", 
            Commands.sequence(
                // 第一步：展开/准备爬升机构，并亮起爬升指示灯
                climber.ClimbingProcessSingleCommand()
                    .alongWith(new InstantCommand(() -> candle.Changecolor(Constants.RobotState.State.ClimbingUp), candle)),
                
                // 第二步：直接设定目标位置
                Commands.runOnce(() -> climber.setPosition(ClimbPosition), climber)
            )
        );

        configureBindings();

        // 同步初始灯光状态，避免启动时 LED 与实际 flag 不一致
        refreshBackgroundState();

        // Build an auto chooser. This will use Commands.none() as the default option.
        autoChooser = AutoBuilder.buildAutoChooser();
        // Another option that allows you to specify the default auto by its name
        // autoChooser = AutoBuilder.buildAutoChooser("My Default Auto");
        SmartDashboard.putData("Auto Chooser", autoChooser);
    }

    //按键绑定
    public void updateDashboard() {
        SmartDashboard.putNumber("Launcher/LaunchSpeed", launchSpeed);
        SmartDashboard.putNumber("Launcher/LaunchAngle", launchAngle);
        SmartDashboard.putNumber("Launcher/SpeedOffset", Constants.ShootingTrim.speedOffset);
        SmartDashboard.putNumber("Launcher/PitchOffset", Constants.ShootingTrim.pitchOffset);
    }
    // Bindings
    private void configureBindings() {
        
        /******************************************************（Driver）**********************************************************************/
        // 这个是默认的开环底盘控制，使用左操纵杆控制平移，右操纵杆控制旋转
        drivetrain.setDefaultCommand(
            drivetrain.applyRequest(() -> {
                // 在归一化轴值（-1~1）上应用死区，保证任何速度模式下推杆响应一致
                double rawX = MathUtil.applyDeadband(-Driver.getLeftY(), 0.1);
                double rawY = MathUtil.applyDeadband(-Driver.getLeftX(), 0.1);
                double rawR = MathUtil.applyDeadband(-Driver.getRightX(), 0.1);
                double scale = getInputScale();
                return drive
                    .withVelocityX(rawX * MaxSpeed * 0.75 * scale)
                    .withVelocityY(rawY * MaxSpeed * 0.75 * scale)
                    .withRotationalRate(rawR * MaxAngularRate * scale);
            })
        );
        drivetrain.registerTelemetry(logger::telemeterize);

        // 在视觉位姿关闭后的定头
        Driver.a().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));

        // 切换是否使用视觉位姿融合
        Driver.b().onTrue(Commands.runOnce(() -> {
            isVisionPoseFusion = !isVisionPoseFusion;
            refreshBackgroundState();
        }));
        // 切换慢速模式（减半速度）
        Driver.start().onTrue(Commands.runOnce(() -> {
            isSlowMode = !isSlowMode;
            refreshBackgroundState();
        }));
                        
        // 手动电推杆微调
        // Driver.povUp().whileTrue(launcher.AdjustAngleSingleCommand(-12));
        // Driver.povDown().whileTrue(launcher.AdjustAngleSingleCommand(12));

        //装福灯
        Driver.y().onTrue(new InstantCommand(()->candle.Changecolor(Constants.RobotState.State.ClimbingDown),candle));


        //调试用途
        // Driver.povRight().onTrue(Commands.runOnce(() -> launchSpeed += 1.25));
        // Driver.povLeft().onTrue(Commands.runOnce(() -> launchSpeed -= 1.25));

        // Driver.povDown().onTrue(Commands.runOnce(() -> launchAngle += 0.001));
        // Driver.povUp().onTrue(Commands.runOnce(() -> launchAngle -= 0.001));
        
        //调试用途
        // Driver.b()
        //     .whileTrue(
        //     Commands.parallel(
        //         new ProxyCommand(() -> ShootingCommand.createShootingCommand(
        //             intake,
        //             launcher,
        //             transport,
        //             launchSpeed,
        //             launchAngle
        //         )),
        //         Commands.runOnce(() -> candle.Changecolor(Constants.RobotState.State.Shooting), candle)
        //     ).finallyDo(() -> candle.restoreBackground())
        // );

        //单独执行发射命令
        Driver.rightTrigger()
            .whileTrue(
            Commands.parallel(
                new ProxyCommand(() -> ShootingCommand.createShootingCommand(
                    intake,
                    launcher,
                    transport,
                    47.5,
                    0.0031
                )),
                Commands.runOnce(() -> candle.Changecolor(Constants.RobotState.State.Shooting), candle)
            ).finallyDo(() -> candle.restoreBackground())
        );

        // Operator A键：中场盲射 Feed，且带intake上下摆动(极短预热跑打，球多需要摆动防止卡球)
        Operator.a()
            .whileTrue(
            Commands.parallel(
                ShootingCommand.createDynamicFeedCommand(
                    intake,
                    launcher,
                    transport,
                    70.0,    // 射速给到最大，保证能飞过半场
                    -0.035,   // 对应适合跨越半场的抛物线角度
                    true // 需要摆动
                ),
                Commands.runOnce(() -> candle.Changecolor(Constants.RobotState.State.Shooting), candle)
            ).finallyDo(() -> candle.restoreBackground())
        );

        // Operator 右扳机键：中场盲射 Feed，且intake不摆动(极短预热跑打，球少不需要摆动)
        Operator.povDown()
            .whileTrue(
            Commands.parallel(
                ShootingCommand.createDynamicFeedCommand(
                    intake,
                    launcher,
                    transport,
                    70.0,    // 射速给到最大，保证能飞过半场
                    -0.035,   // 对应适合跨越半场的抛物线角度
                    false // 不需要摆动
                ),
                Commands.runOnce(() -> candle.Changecolor(Constants.RobotState.State.Shooting), candle)
            ).finallyDo(() -> candle.restoreBackground())
        );

        //intake机构放下or回收
        Driver.x().onTrue(intake.ChangePitchPositionSingleCommand()
                    .andThen(Commands.either(
                        intake.AdjustIntakePositionSingleCommand(IntakeUpPosition), 
                        intake.AdjustIntakePositionSingleCommand(IntakeDownPosition), 
                        ()->intake.IntakepitchPositionFlag))
                        );

        //吸球
        Driver.rightBumper().onTrue(
            intake.ChangeIntakeSpeedSingleCommand()
            .andThen(intake.IntakeSingleCommand())
            .andThen(Commands.either(
                new InstantCommand(() -> candle.Changecolor(Constants.RobotState.State.Intaking), candle),
                new InstantCommand(() -> candle.restoreBackground(), candle),
                () -> intake.Intake_press_times % 2 == 1
            ))
            
        );

        //吐球
        Driver.leftBumper().whileTrue(OuttakeCommand.create(intake, launcher, transport, candle)
                            .alongWith(new InstantCommand(()->candle.Changecolor(Constants.RobotState.State.Outtaking),candle)));

        /*****************************************************（Operator）**********************************************************/
        // Operator.a().whileTrue(
        //     new OuttakeCommand(intake, launcher, transport)
            
        // );
        // Operator.povRight().whileTrue(
        //     Commands.runOnce(() -> {
        //         System.out.println("SWING");
        //     })
        //     .andThen(
        //         Commands.runOnce(()->
        //             intake.IntakeSwingSingleCommand().repeatedly())
        //     ));

        
        //爬升
        Operator.back().onTrue(climber.ClimbingProcessSingleCommand());
        Operator.start().whileTrue(
            Commands.run(() -> climber.setPosition(ClimbPosition))
        ).onFalse(
            Commands.sequence(
                Commands.runOnce(()->climber.releaseClimber()),
                Commands.waitSeconds(0.1),
                Commands.runOnce(() -> climber.setPosition(climber.getCurrentPosition()))
            )
        );


        // 半自动点位近左
        Operator.x().whileTrue(
            Commands.runOnce(() -> {
                System.out.println("Starting Hub targeting command");
                isVisionPoseFusion = true; // 进入半自动模式，开启视觉位姿融合
                candle.Changecolor(Constants.RobotState.State.Shooting);
            })
            .andThen(MagicSequencingCommand.createFixedPointAutoScoreCommand(
                0,
                drivetrain, 
                intake, 
                launcher, 
                transport,
                Constants.VisionConfig.BLUE_HUB_CENTER,
                Constants.VisionConfig.POINTS_PARAMS_TABLE_BLUE
            ))
            .finallyDo((interrupted) -> {
                //isVisionPoseFusion = false; // 退出半自动模式，关闭视觉位姿融合
                candle.restoreBackground();
                launcher.setFrictionWheelVelocity(0);//防止半自动预热后被中断导致摩擦轮一直转
                System.out.println("Hub targeting command ended. Interrupted: " + interrupted);
            })
        );

        //半自动点位近中
        Operator.y().whileTrue(
            Commands.runOnce(() -> {
                System.out.println("Starting Hub targeting command");
                isVisionPoseFusion = true;
                candle.Changecolor(Constants.RobotState.State.Shooting);
            })
            .andThen(MagicSequencingCommand.createFixedPointAutoScoreCommand(
                1,
                drivetrain, 
                intake, 
                launcher, 
                transport,
                Constants.VisionConfig.BLUE_HUB_CENTER,
                Constants.VisionConfig.POINTS_PARAMS_TABLE_BLUE
            ))
            .finallyDo((interrupted) -> {
                //isVisionPoseFusion = false; // 退出半自动模式，关闭视觉位姿融合
                candle.restoreBackground();
                launcher.setFrictionWheelVelocity(0);//防止半自动预热后被中断导致摩擦轮一直转
                System.out.println("Hub targeting command ended. Interrupted: " + interrupted);
            })
        );

        //半自动点位近右
        Operator.b().whileTrue(
            Commands.runOnce(() -> {
                System.out.println("Starting Hub targeting command");
                isVisionPoseFusion = true;
                candle.Changecolor(Constants.RobotState.State.Shooting);
            })
            .andThen(MagicSequencingCommand.createFixedPointAutoScoreCommand(
                2,
                drivetrain, 
                intake, 
                launcher, 
                transport,
                Constants.VisionConfig.BLUE_HUB_CENTER,
                Constants.VisionConfig.POINTS_PARAMS_TABLE_BLUE
            ))
            .finallyDo((interrupted) -> {
                //isVisionPoseFusion = false; // 退出半自动模式，关闭视觉位姿融合
                candle.restoreBackground();
                launcher.setFrictionWheelVelocity(0);//防止半自动预热后被中断导致摩擦轮一直转
                System.out.println("Hub targeting command ended. Interrupted: " + interrupted);
            })
        );
                //半自动点位远左
        Operator.povLeft().whileTrue(
            Commands.runOnce(() -> {
                System.out.println("Starting Hub targeting command");
                isVisionPoseFusion = true;
                candle.Changecolor(Constants.RobotState.State.Shooting);
            })
            .andThen(MagicSequencingCommand.createFixedPointAutoScoreCommand(
                3,
                drivetrain, 
                intake, 
                launcher, 
                transport,
                Constants.VisionConfig.BLUE_HUB_CENTER,
                Constants.VisionConfig.POINTS_PARAMS_TABLE_BLUE
            ))
            .finallyDo((interrupted) -> {
                //isVisionPoseFusion = false; // 退出半自动模式，关闭视觉位姿融合
                candle.restoreBackground();
                launcher.setFrictionWheelVelocity(0);//防止半自动预热后被中断导致摩擦轮一直转
                System.out.println("Hub targeting command ended. Interrupted: " + interrupted);
            })
        );

        //半自动点位远中
        Operator.povUp().whileTrue(
            Commands.runOnce(() -> {
                System.out.println("Starting Hub targeting command");
                isVisionPoseFusion = true;
                candle.Changecolor(Constants.RobotState.State.Shooting);
            })
            .andThen(MagicSequencingCommand.createFixedPointAutoScoreCommand(
                4,
                drivetrain, 
                intake, 
                launcher, 
                transport,
                Constants.VisionConfig.BLUE_HUB_CENTER,
                Constants.VisionConfig.POINTS_PARAMS_TABLE_BLUE
            ))
            .finallyDo((interrupted) -> {
                //isVisionPoseFusion = false; // 退出半自动模式，关闭视觉位姿融合
                candle.restoreBackground();
                launcher.setFrictionWheelVelocity(0);//防止半自动预热后被中断导致摩擦轮一直转
                System.out.println("Hub targeting command ended. Interrupted: " + interrupted);
            })
        );

        //半自动点位远右
        Operator.povRight().whileTrue(
            Commands.runOnce(() -> {
                System.out.println("Starting Hub targeting command");
                isVisionPoseFusion = true;
                candle.Changecolor(Constants.RobotState.State.Shooting);
            })
            .andThen(MagicSequencingCommand.createFixedPointAutoScoreCommand(
                5,
                drivetrain, 
                intake, 
                launcher, 
                transport,
                Constants.VisionConfig.BLUE_HUB_CENTER,
                Constants.VisionConfig.POINTS_PARAMS_TABLE_BLUE
            ))
            .finallyDo((interrupted) -> {
                //isVisionPoseFusion = false; // 退出半自动模式，关闭视觉位姿融合
                candle.restoreBackground();
                launcher.setFrictionWheelVelocity(0);//防止半自动预热后被中断导致摩擦轮一直转
                System.out.println("Hub targeting command ended. Interrupted: " + interrupted);
            })
        );

        // 比赛现场全局射击参数微调（整体偏移，叠加在所有查表结果之上）
        // POV上下 = 射速偏移 ±1.25 rps；POV左右 = Pitch角度偏移 ±0.001 rot
        Operator.rightBumper().onTrue(Commands.runOnce(() -> {
            Constants.ShootingTrim.speedOffset += 1.25;
            System.out.println("[Trim] SpeedOffset=" + Constants.ShootingTrim.speedOffset + " PitchOffset=" + Constants.ShootingTrim.pitchOffset);
        }));
        Operator.rightTrigger().onTrue(Commands.runOnce(() -> {
            Constants.ShootingTrim.speedOffset -= 1.25;
            System.out.println("[Trim] SpeedOffset=" + Constants.ShootingTrim.speedOffset + " PitchOffset=" + Constants.ShootingTrim.pitchOffset);
        }));
        Operator.leftBumper().onTrue(Commands.runOnce(() -> {
            Constants.ShootingTrim.pitchOffset += 0.001;
            System.out.println("[Trim] SpeedOffset=" + Constants.ShootingTrim.speedOffset + " PitchOffset=" + Constants.ShootingTrim.pitchOffset);
        }));
        Operator.leftTrigger().onTrue(Commands.runOnce(() -> {
            Constants.ShootingTrim.pitchOffset -= 0.001;
            System.out.println("[Trim] SpeedOffset=" + Constants.ShootingTrim.speedOffset + " PitchOffset=" + Constants.ShootingTrim.pitchOffset);
        }));

        // //半自动原地瞄准
        // Driver.leftTrigger().whileTrue(
        //     Commands.runOnce(() -> {
        //         if(intake.Intake_press_times % 2 == 1){intake.ChangeIntakeSpeedSingleCommand();}
        //         System.out.println("Starting Hub targeting command");
        //         isVisionPoseFusion = true;
        //     })
        //     .andThen(new InstantCommand(() -> candle.Changecolor(Constants.RobotState.State.Shooting), candle))
        //     .andThen(MagicSequencingCommand.createAnyPointAutoScoreCommand(
        //         drivetrain, 
        //         intake, 
        //         launcher, 
        //         transport,
        //         Constants.VisionConfig.BLUE_HUB_CENTER
        //     ))
        //     .finallyDo((interrupted) -> {
        //         candle.restoreBackground();
        //         //isVisionPoseFusion = false; // 退出半自动模式，关闭视觉位姿融合
        //         launcher.setFrictionWheelVelocity(0);//防止半自动预热后被中断导致摩擦轮一直转
        //         System.out.println("Hub targeting command ended. Interrupted: " + interrupted);
        //         new InstantCommand(() -> candle.Changecolor(Constants.RobotState.State.Shooting));
        //     })
        // );

        // 操作手按住左扳机时：原地转向 + 动态发射（参考 MagicSequencingCommand 原地发射逻辑）
        Driver.leftTrigger().whileTrue(
            Commands.parallel(MoveWhileAimCommand.create(
                    drivetrain,
                    () -> -Driver.getLeftY() * MaxSpeed * 0.5 * getInputScale(),
                    () -> -Driver.getLeftX() * MaxSpeed * 0.4 * getInputScale(),
                    MaxAngularRate * getInputScale(),
                    Constants.VisionConfig.BLUE_HUB_CENTER
                    ),
                    ShootingCommand.createDynamicShootingCommand(
                        drivetrain,
                        intake,
                        launcher,
                        transport,
                        Constants.VisionConfig.BLUE_HUB_CENTER
                    )
                )
                .beforeStarting(() -> candle.Changecolor(Constants.RobotState.State.Shooting))
                .finallyDo(() -> candle.restoreBackground())
        );
            
            
            

        //********************************************************** (Sysidroutine) ******************************************************
        // // Run SysId routines when holding back/start and X/Y.
        // // Note that each routine should be run exactly once in a single log.
        // Driver.back().and(Driver.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
        // Driver.back().and(Driver.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
        // Driver.start().and(Driver.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
        // Driver.start().and(Driver.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

    }
    
    //融合视觉位姿
    public void addMeasurements() {

        SwerveDriveState driveState = drivetrain.getState();
        List<VisionMeasurement> measurements = vision.processVisionData(driveState);

        for (VisionMeasurement m : measurements) {
            drivetrain.addVisionMeasurement(m.pose, m.timestamp, m.stdDevs);
        }

    }

    //初始重置odom位置，防止启动区看不到tag导致乱跑
    public Command getAutoInitCommand(){
        return AutoBuilder.resetOdom(Constants.VisionConfig.m_initialPose);
    }

    //返回选择的auto命令
    public Command getAutonomousCommand() {
        // This method loads the auto when it is called, however, it is recommended
        // to first load your paths/autos when code starts, then return the
        // pre-loaded auto/path
        return autoChooser.getSelected();
    }

    // 根据当前慢速模式状态返回输入缩放系数，慢速模式下为0.25，正常模式下为1.0
    private double getInputScale() {
        return isSlowMode ? 0.25 : 1.0;
    }

    /**
     * 根据当前所有持久状态的组合刷新背景灯光。
     * 优先级：SlowMode（青色）> VisionFusion（黄色）> Idle（灭灯）
     * 在任何持久状态切换后调用，保证不同组合下灯光正确。
     */
    private void refreshBackgroundState() {
        if (isSlowMode) {
            candle.setBackgroundState(Constants.RobotState.State.ClimbingUp);
        } else if (!isVisionPoseFusion) {
            // vision 关闭时亮黄色警告
            candle.setBackgroundState(Constants.RobotState.State.VisionFusion);
        } else {
            candle.setBackgroundState(Constants.RobotState.State.Idle);
        }
    }
}
