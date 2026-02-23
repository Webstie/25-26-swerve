package frc.robot;

import java.util.Arrays;
import java.util.List;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.XboxController;

public class Constants {

    //variables for robot state
    public static class RobotState{
        public enum State {
            Shooting, Intaking, ClimbingUp, Outtaking, ClimbingDown, Idle;
        };
    }

    //candle Constants
    public static class CANDLE{
        public static final int CANdleID1 = 1;
        public static final int CANdleID2 = 2;
        public static final int JoystickId = 0;
        public static final int IncrementAnimButton = XboxController.Button.kRightBumper.value;
        public static final int DecrementAnimButton = XboxController.Button.kLeftBumper.value;
        public static final int BlockButton = XboxController.Button.kStart.value;
        public static final int MaxBrightnessAngle = 90;
        public static final int MidBrightnessAngle = 180;
        public static final int ZeroBrightnessAngle = 270;
        public static final int VbatButton = XboxController.Button.kA.value;
        public static final int V5Button = XboxController.Button.kB.value;
        public static final int CurrentButton = XboxController.Button.kX.value;
        public static final int TemperatureButton = XboxController.Button.kY.value;
    }

    public static final class ClimberConfig{
        public static final double ClimberTopPosition = -120.0;
        public static final double ClimbPosition = 1.0;
        public static final int CLIMBER_MOTOR_ID = 1;
    }

    public static final class LauncherConfig {
        public static final double Near_FrictionWheelLaunchSpeed = 50;///////////////////////////////////////////////////
        public static final double Far_FrictionWheelLaunchSpeed = 53.5;///////////////////////////////////////////////////

        //远处近处不同点位对应的电推杆角度
        public static final double Near_launch_angle = -0.0015;
        public static final double Far_launch_angle  = -0.032;

        public static final double WarmupSecond = 3;////////////////////////////////////////////////////////////////
        public static final double FeederSpeed = 55.0;/////////////////////////////////////////////////////////////////
        public static final double OuttakeBallspeed = -30.0;
        public static final double shootingVoltage = 12.0;
        public static final double FrictionWheelVelocityRampRate = 50.0;

        public static final int SHOOTER_SPARKMAX_ID = 9;
        public static final int ANGLE_CANCODER_ID = 1;
        public static final int FEEDER_MOTOR_ID = 2;
        public static final int LEFT_FRICTIONWHEEL_MOTOR_ID = 3;
        public static final int MIDDLE_FRICTIONWHEEL_MOTOR_ID = 4;
        public static final int RIGHT_FRICTIONWHEEL_MOTOR_ID = 5;
    }

    public static final class IntakeConfig{
        public static final double IntakeSwingDownPosition = -15.0;
        public static final double IntakeSwingUpPosition = -13.0;
        public static final double IntakeUpPosition = 0.0;
        public static final double IntakeDownPosition = -16.5;
        public static final double SwingWaitTime = 0.1;
        public static final double OuttakeVelocity = 20.0;
        public static final double IntakeVelocity = -70.0;
        public static final double SupportVelocity = 60.0;//////////////////////////////////////////////////////////(可以比较快)
    
        public static final int INTAKE_MOTOR_ID = 6;
        public static final int INTAKE_PITCH_MOTOR_ID = 9;
        public static final int INTAKE_SUPPORT_MOTOR_ID = 7;
    }

    public static final class TransportConfig{
        public static double  TransportSpeed = 50.0;/////////////////////////////////////////////////////////////////////
        
        public static int TRANSPORT_MOTOR_ID = 10;
    }

    public static final class Candle{
        public static final int CANDLEID = 1;
    }

    public static class VisionConfig {
        // The layout of the AprilTags on the field
        public static final AprilTagFieldLayout aprilTagFieldLayout = AprilTagFieldLayout
                .loadField(AprilTagFields.kDefaultField);
        public static Pose2d m_initialPose = new Pose2d(3.67, 0.67, Rotation2d.fromDegrees(180)); //blue start
        //public static Pose2d m_initialPose = new Pose2d(7.68, 3.7, Rotation2d.fromDegrees(180)); //red start

        //到点容差半径，单位英寸
        public static final double SCORING_SIDE_RADIUS_ROBOT_IN = 18.25;

        //两侧挂珊瑚位置到tag中心偏移
        public static final double TAG_TO_BRANCH_OFFSET_M = 0.17;
        
        //对正时的角度
        public static final Rotation2d SCORING_SIDE_FROM_FRONT_ROT = new Rotation2d(Math.PI);

        // 蓝色联盟视角下的Hub center点和周围得分点（单位：米）
        public static final Translation2d BLUE_HUB_CENTER = new Translation2d(4.626, 4.035);
        // 假设周围三个点
        public static final List<Translation2d> BLUE_SCORING_NODES = Arrays.asList(
            // new Translation2d(2.6, 2.7),
            // new Translation2d(2.7, 1.6),
            // new Translation2d(2.5, 3.3)

            //近处3个点
            new Translation2d(3.3, 5.4),//近左（0
            new Translation2d(2.8, 4.03),//近中（1
            new Translation2d(3.2, 2.6),//近右（2
  

            //远处三个点
            new Translation2d(2.17, 6.01),//远左（3
            new Translation2d(1.642, 3.61),//远中（4
            new Translation2d(2.039, 2.065)//远右（5
        );

        //不同距离映射的pitch角度
        public static final double[][] DISTANCE_DATA_TABLE = {
            // {距离, Pitch角度,射速}
            {0.375, -0.0015,50}, // 第 1 点
            {0.750, -0.0015,50}, // 第 2 点
            {1.125, -0.0015,50}, // 第 3 点
            {1.5, -0.0015,50},   // 第 4 点
            {1.8, -0.0015,50}, // 第 5 点*
            {2.25, -0.00071,55},  // 第 6 点*
            {2.625, -0.004,56.5}, // 第 7 点*
            {3.0, -0.007,59.5},   // 第 8 点
            {3.3, -0.007,60}, // 第 9 点
            {3.83, -0.0015,50},  // 第 10 点
            {4.2, -0.032,60}, // 第 11 点
            {4.5, -0.032,50},   // 第 12 点
            {4.875, -0.032,50}, // 第 13 点
            {5.25, -0.032,50},  // 第 14 点
            {5.625, -0.032,50}, // 第 15 点
            {6.0, -0.032,50},   // 第 16 点
            {6.375,-0.032,50}, // 第 17 点
            {6.75, -0.032,50},  // 第 18 点
            {7.125, -0.032,50}, // 第 19 点
            {7.5, -0.032,50}    // 第 20 点
        };


        //目前测量得出，x方向运动误差大约为+0.1左右，y方向较小，故调整容差在同一数量级稍小，使位置控制更稳定
        public static final double LINEUP_TOLERANCE_METERS = 0.015;

        public static final double ANGLE_TOLERANCE_DEGREES = 0.8;
    }
}
