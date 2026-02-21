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
            STATE1, STATE2, STATE3, STATE4;
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
        public static final double FrictionWheelLaunchSpeed = 55.0;
        public static final double WarmupSecond = 2.0;
        public static final double FeederSpeed = 50.0;
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
        public static final double IntakeSwingDownPosition = -14.0;
        public static final double IntakeSwingUpPosition = -11.0;
        public static final double IntakeUpPosition = 0.0;
        public static final double IntakeDownPosition = -16.5;
        public static final double SwingWaitTime = 0.25;
        public static final double OuttakeVelocity = 50.0;
        public static final double IntakeVelocity = -50.0;
        
        public static final int INTAKE_MOTOR_ID = 6;
        public static final int INTAKE_PITCH_MOTOR_ID = 7;
        public static final int INTAKE_SUPPORT_MOTOR_ID = 9;
    }

    public static final class TransportConfig{
        public static int TransportSpeed = 40;
        
        public static int TRANSPORT_MOTOR_ID = 8;
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

            new Translation2d(3.2, 2.6),
            new Translation2d(2.8, 3.9)
        );

        //目前测量得出，x方向运动误差大约为+0.1左右，y方向较小，故调整容差在同一数量级稍小，使位置控制更稳定
        public static final double LINEUP_TOLERANCE_METERS = 0.015;

        public static final double ANGLE_TOLERANCE_DEGREES = 1.0;
    }
}