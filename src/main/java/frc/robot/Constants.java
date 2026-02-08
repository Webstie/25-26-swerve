package frc.robot;

import java.util.HashMap;
import java.util.Map;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.XboxController;

public class Constants {

    //variables for robot state
    public static class RobotState{
        public enum State {
            STATE1, STATE2, STATE3, STATE4;
        };
    }

    //candel Constants
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
    public static final class Climber{
        public static final double ClimberTopPosition = -120.0;
        public static final double ClimbPosition = 1.0;

        public static final int CLIMBER_MOTOR_ID = 1;
    }

    public static final class Shooter {
        public static final double Frictionwheelshootspeed = 20.0;
        public static final double Intakeballspeed = 30.0;
        public static final double OuttakeBallspeed = -30.0;
        public static final double shootingVoltage = 12.0;
        public static final double FrictionwheelVelocityRampRate = 50.0;

        public static final int SHOOTER_SPARKMAX_ID = 9;
        public static final int FEEDER_MOTOR_ID = 2;
        public static final int LEFT_FRICTIONWHEEL_MOTOR_ID = 3;
        public static final int MIDDLE_FRICTIONWHEEL_MOTOR_ID = 4;
        public static final int RIGHT_FRICTIONWHEEL_MOTOR_ID = 5;
    }

    public static final class Intake{
        public static final double IntakeSwingDownPosition = -14.0;
        public static final double IntakeSwingUpPosition = -10.0;
        public static final double IntakeUpPosition = 0.0;
        public static final double IntakeDownPosition = -16.5;
        public static final double SwingWaitTime = 0.25;
        public static final double OuttakeVelocity = 50.0;
        public static final double IntakeVelocity = -50.0;
        
        public static final int INTAKE_MOTOR_ID = 6;
        public static final int INTAKE_PITCH_MOTOR_ID = 7;
    }

    public static final class Transport{
        public static int TransportSpeed = 40;

        public static int TRANSPORT_MOTOR_ID = 8;
    }

    public static final class Candle{
        public static final int CANDLEID = 1;
    }

    public static class Vision {
        
        // The layout of the AprilTags on the field
        public static final AprilTagFieldLayout aprilTagFieldLayout = AprilTagFieldLayout
                .loadField(AprilTagFields.kDefaultField);
        public static Pose2d m_initialPose = new Pose2d(7.68, 3.7, Rotation2d.fromDegrees(180)); //blue start middle
        // public Pose2d m_initialPose = new Pose2d(7.68, 3.7, Rotation2d.fromDegrees(180)); //blue start left
        // public Pose2d m_initialPose = new Pose2d(7.68, 3.7, Rotation2d.fromDegrees(180)); //blue start right

        //public static Pose2d m_initialPose = new Pose2d(7.68, 3.7, Rotation2d.fromDegrees(180)); //red start middle
        // public Pose2d m_initialPose = new Pose2d(7.68, 3.7, Rotation2d.fromDegrees(180)); //red start left
        // public Pose2d m_initialPose = new Pose2d(7.68, 3.7, Rotation2d.fromDegrees(180)); //red start right

        // The AprilTag IDs for scoring points
        public static final Map<Integer, String> hubTagNames = new HashMap<>(){{
                put(6, "8oC");
                put(7, "6oC");
                put(8, "4oC");
                put(9, "2oC");
                put(10, "12oC");
                put(11, "10oC");
                put(17, "4oC");
                put(18, "6oC");
                put(19, "8oC");
                put(20, "10oC");
                put(21, "12oC");
                put(22, "2oC");
            }};

        //到点容差半径
        public static final double SCORING_SIDE_RADIUS_ROBOT_IN = 18.25;

        //两侧挂珊瑚位置到tag中心偏移
        public static final double TAG_TO_BRANCH_OFFSET_M = 0.17;
        
        //对正时的角度
        public static final Rotation2d SCORING_SIDE_FROM_FRONT_ROT = new Rotation2d(Math.PI);
    
    }
}
