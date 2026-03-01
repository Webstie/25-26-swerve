package frc.robot;

import java.util.Arrays;
import java.util.List;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
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
        public static final double ClimberTopPosition = 120.0;
        public static final double ClimbPosition = 1.0;
        public static final int CLIMBER_MOTOR_ID = 1;
    }

    public static final class LauncherConfig {
        public static final double WarmupSecond = 1.0;
        public static final double FeederSpeed = 55.0;
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
        public static final double IntakeVelocity = -85.0;
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
        public static final AprilTagFieldLayout aprilTagFieldLayout = AprilTagFieldLayout
                .loadField(AprilTagFields.kDefaultField);

        //blue start
        public static Pose2d m_initialPose = new Pose2d(3.67, 0.67, Rotation2d.fromDegrees(180));

        //red start
        //public static Pose2d m_initialPose = new Pose2d(7.68, 3.7, Rotation2d.fromDegrees(180));

        //位置容差半径，单位英寸
        public static final double SCORING_SIDE_RADIUS_ROBOT_IN = 18.25;
        
        //对正时的角度
        public static final Rotation2d SCORING_SIDE_FROM_FRONT_ROT = new Rotation2d(Math.PI);

        //蓝色联盟视角下的Hub center点和周围得分点（单位：米）
        public static final Translation2d BLUE_HUB_CENTER = new Translation2d(4.626, 4.035);

        //供自动和固定点位自瞄发射时使用的点位与射击参数的映射表
        public static final double[][] POINTS_PARAMS_TABLE_BLUE = {
            // {距离, Pitch角度,射速}
            {3.3, 5.4, -0.0015, 50}, // 第 1 点
            {2.8, 4.03, -0.0015, 50},   // 第 2 点
            {3.2, 2.6, -0.0015, 50}, // 第 3 点

            {2.17, 6.01, -0.015, 58.5},  // 第 4 点
            {1.642, 3.61, -0.015, 58.5}, // 第 5 点
            {2.039, 2.065, -0.015, 58.5},   // 第 6 点
        };

        //TODO红色还没测
        public static final double[][] POINTS_PARAMS_TABLE_RED = {
            // {距离, Pitch角度,射速}
            {3.3, 5.4, -0.0015, 50}, // 第 1 点
            {2.8, 4.03, -0.0015, 50},   // 第 2 点
            {3.2, 2.6, -0.0015, 50}, // 第 3 点

            {2.17, 6.01, -0.032, 53.5},  // 第 4 点
            {1.642, 3.61, -0.032, 53.5}, // 第 5 点
            {2.039, 2.065, -0.032, 53.5},   // 第 6 点
        };

        //供任意点位原地自瞄发射使用的距离与射击参数的映射表
        public static final double[][] DISTANCE_PARAMS_TABLE = {
            // {距离, Pitch角度,射速}
            {1.125, 0.0031, 47.5}, // 第 3 点
            {1.5, 0.0029, 50},   // 第 4 点
            {1.875, -0.001, 50}, // 第 5 点
            {2.25, -0.008, 53.75},  // 第 6 点
            {2.625, -0.014, 55}, // 第 7 点
            {3.0, -0.015, 57.5},   // 第 8 点
            {3.375, -0.015, 60}, // 第 9 点
            {3.75, -0.016, 62.5},  // 第 10 点
            {4.125, -0.020, 63.75}, // 第 11 点
            {4.5, -0.022, 67.5},   // 第 12 点
            {4.875, -0.025, 69}, // 第 13 点
            {5.25, -0.025, 70},  // 第 14 点
        };
        
        // ==========================================
        // [新增] 声明两个全局静态的插值表
        // ==========================================
        public static final InterpolatingDoubleTreeMap distanceToPitchMap = new InterpolatingDoubleTreeMap();
        public static final InterpolatingDoubleTreeMap distanceToSpeedMap = new InterpolatingDoubleTreeMap();

        // ==========================================
        // [新增] 静态初始化块 (Static Initialization Block)
        // 当程序启动，JVM加载 Constants 类时，这段代码会自动且只执行一次。
        // 它会自动遍历上面的 DISTANCE_PARAMS_TABLE，把数据塞进插值表里。
        // ==========================================
        static {
            for (double[] point : DISTANCE_PARAMS_TABLE) {
                double distance = point[0];
                double pitch = point[1];
                double speed = point[2];
                
                distanceToPitchMap.put(distance, pitch);
                distanceToSpeedMap.put(distance, speed);
            }
        }

        //目前测量得出，x方向运动误差大约为+0.1左右，y方向较小，故调整容差在同一数量级稍小，使位置控制更稳定
        public static final double LINEUP_TOLERANCE_METERS = 0.015;
        public static final double ANGLE_TOLERANCE_DEGREES = 4;
        
        public static final double PitchSlope = -0.00843836;
        public static final double PitchYIntercept = 0.0133425;
        public static final double SpeedSlope = 5.69635;
        public static final double SpeedYIntercept = 40.73202;
    }
}
