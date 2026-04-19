package frc.robot;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.wpilibj.XboxController;

public class Constants {

    public static class RobotState {
        public enum State {
            Shooting, Intaking, ClimbingUp, Outtaking, ClimbingDown, Idle, VisionFusion;
        }
    }

    public static class Layout {
        public static final double FIELD_LENGTH_METERS = 16.540988;
        public static final double FIELD_WIDTH_METERS = 8.0692752;
    }

    public static class CANDLE {
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

    public static final class ClimberConfig {
        public static final double ClimberTopPosition = 95.0;
        public static final double ClimbPosition = 1.0;
        public static final int CLIMBER_MOTOR_ID = 1;
    }

    public static final class LauncherConfig {
        public static final double WarmupSpeed = 60.0;
        public static final double WarmupSecond = 1.0;
        public static final double FeederSpeed = 100.0;
        public static final double OuttakeBallspeed = -30.0;
        public static final double FrictionWheelOuttakeVelocity = -40.0;
        public static final double shootingVoltage = 12.0;
        public static final double FrictionWheelVelocityRampRate = 50.0;

        // Driver right-trigger manual shoot parameters
        public static final double ManualShootSpeed = 47.5;
        public static final double ManualShootAngle = 0.0031;

        // Operator mid-field feed parameters (A button / povDown)
        public static final double MidFieldFeedSpeed = 70.0;
        public static final double MidFieldFeedAngle = -0.035;

        // Warmup durations
        public static final double FastWarmupSeconds = 0.5;
        public static final double AutoDynamicWarmupSeconds = 4.0;

        public static final int SHOOTER_SPARKMAX_ID = 9;
        public static final int ANGLE_CANCODER_ID = 1;
        public static final int FEEDER_MOTOR_ID = 2;
        public static final int LEFT_FRICTIONWHEEL_MOTOR_ID = 3;
        public static final int MIDDLE_FRICTIONWHEEL_MOTOR_ID = 4;
        public static final int RIGHT_FRICTIONWHEEL_MOTOR_ID = 5;
    }

    public static final class IntakeConfig {
        public static final double IntakeSwingDownPosition = -15.0;
        public static final double IntakeSwingUpPosition = -13.0;
        public static final double IntakeUpPosition = 0.0;
        public static final double IntakeDownPosition = -17.5;
        public static final double SwingWaitTime = 0.1;
        public static final double OuttakeVelocity = 20.0;
        public static final double IntakeVelocity = -90.0;

        public static final int INTAKE_LEFT_MOTOR_ID = 6;
        public static final int INTAKE_RIGHT_MOTOR_ID = 7;
        public static final int INTAKE_PITCH_MOTOR_ID = 9;
    }

    public static final class TransportConfig {
        public static final double TransportSpeed = 90.0;
        public static final int TRANSPORT_MOTOR_ID = 10;
    }

    public static final class DriveConfig {
        public static final double TeleopDriveSpeedScale = 0.75;
        public static final double AimDriveScaleX = 0.5;
        public static final double AimDriveScaleY = 0.4;
    }

    public static class VisionConfig {
        public static final AprilTagFieldLayout aprilTagFieldLayout = AprilTagFieldLayout
                .loadField(AprilTagFields.kDefaultField);

        // Blue alliance start pose
        public static Pose2d m_initialPose = new Pose2d(3.67, 0.67, Rotation2d.fromDegrees(180));

        public static final double SCORING_SIDE_RADIUS_ROBOT_IN = 18.25;
        public static final Rotation2d SCORING_SIDE_FROM_FRONT_ROT = new Rotation2d(Math.PI);

        // Hub center and scoring positions (blue alliance frame, meters)
        public static final Translation2d BLUE_HUB_CENTER = new Translation2d(4.626, 4.035);

        // Fixed scoring positions: indices 0-2 = near (left/mid/right), 3-5 = far (left/mid/right)
        // Columns: {distance, field_y, pitch_rot, speed_rps}
        public static final double[][] POINTS_PARAMS_TABLE_BLUE = {
            {3.3,   5.4,   -0.0015, 50},   // Point 1 (near left)
            {2.8,   4.03,  -0.0015, 50},   // Point 2 (near mid)
            {3.2,   2.6,   -0.0015, 50},   // Point 3 (near right)
            {2.17,  6.01,  -0.015,  58.5}, // Point 4 (far left)
            {1.642, 3.61,  -0.015,  58.5}, // Point 5 (far mid)
            {2.039, 2.065, -0.015,  58.5}, // Point 6 (far right)
        };

        // Distance-based interpolation table for dynamic shooting
        // Columns: {distance_m, pitch_rot, speed_rps}
        public static final double[][] DISTANCE_PARAMS_TABLE = {
            // {1.125, 0.0031,  47.5},  // Point 1
            // {1.5,   0.0029,  50},    // Point 2
            // {1.875, -0.001,  50},    // Point 3
            // {2.25,  -0.008,  53.75}, // Point 4
            // {2.625, -0.014,  55},    // Point 5
            // {3.0,   -0.015,  57.5},  // Point 6
            // {3.375, -0.015,  60},    // Point 7
            // {3.75,  -0.016,  62.5},  // Point 8
            // {4.125, -0.020,  63.75}, // Point 9
            // {4.5,   -0.022,  65.5},  // Point 10
            // {4.875, -0.025,  67},    // Point 11
            // {5.25,  -0.025,  69},    // Point 12
            // {5.625, -0.025,  70.25}, // Point 13
            {1.25, -0.0000,  45.00},  // Point 1
            {1.50, -0.0000,  47.50},    // Point 2
            {1.75, -0.0000,  49.50},    // Point 3
            {2.00, -0.0015,  51.50}, // Point 4
            {2.25, -0.0020,  52.00},    // Point 5
            {2.50, -0.0040,  54.50},  // Point 6
            {2.75, -0.0055,  55.25},    // Point 7
            {3.00, -0.0065,  56.50},  // Point 8
            {3.25, -0.0090,  59.00}, // Point 9
            {3.50, -0.0100,  60.00},  // Point 10
            {3.75, -0.0105,  60.25},    // Point 11
            {4.00, -0.0120,  61.50},    // Point 12
            {4.25, -0.0125,  61.75}, // Point 13
            {4.50, -0.0135,  62.25},  // Point 14
            {4.75, -0.0170,  64.00},    // Point 15
            {5.00, -0.0180,  65.00},    // Point 16
            {5.25, -0.0190,  66.00}, // Point 7
            {5.50, -0.0200,  66.50},  // Point 18
        };

        public static final InterpolatingDoubleTreeMap distanceToPitchMap = new InterpolatingDoubleTreeMap();
        public static final InterpolatingDoubleTreeMap distanceToSpeedMap = new InterpolatingDoubleTreeMap();

        static {
            for (double[] point : DISTANCE_PARAMS_TABLE) {
                distanceToPitchMap.put(point[0], point[1]);
                distanceToSpeedMap.put(point[0], point[2]);
            }
        }

        // Corner feed targets — blue alliance loading station corners (blue alliance frame)
        public static final Translation2d BLUE_CORNER_LEFT  = new Translation2d(1.5, 7.0);
        public static final Translation2d BLUE_CORNER_RIGHT = new Translation2d(1.5, 1.0);

        // Reject corner feed if robot is closer than this to the hub (ball won't clear)
        public static final double CORNER_FEED_MIN_HUB_DISTANCE = 0.5;

        // Corner feed distance → {pitch_rot, speed_rps} — long lob shots
        public static final double[][] CORNER_FEED_DISTANCE_TABLE = {
            {5.0,  -0.015,  70.0},
            {6.0,  -0.020,  75.0},
            {7.0,  -0.025,  80.0},
            {8.0, -0.030,  85.0},
        };

        public static final InterpolatingDoubleTreeMap distanceToCornerPitchMap = new InterpolatingDoubleTreeMap();
        public static final InterpolatingDoubleTreeMap distanceToCornerSpeedMap = new InterpolatingDoubleTreeMap();

        static {
            for (double[] point : CORNER_FEED_DISTANCE_TABLE) {
                distanceToCornerPitchMap.put(point[0], point[1]);
                distanceToCornerSpeedMap.put(point[0], point[2]);
            }
        }

        public static final double LINEUP_TOLERANCE_METERS = 0.015;
        public static final double ANGLE_TOLERANCE_DEGREES = 3.0;
    }

    /** Global in-match shooting trim: additive offsets applied on top of all table lookups. */
    public static class ShootingTrim {
        public static final double SPEED_TRIM_STEP = 1.25;
        public static final double PITCH_TRIM_STEP = 0.001;
        /** Speed offset (rps): positive = faster */
        public static double speedOffset = 0.0;
        /** Pitch offset (rotations): positive = higher angle */
        public static double pitchOffset = 0.0;
    }

    public static class KalmanFilterConfig {
        public static final double kDt = 0.020; // 20ms
        public static final double[] stateStdDevs = {0.01, 0.1, 0.5};
        public static final double[] measurementStdDevs = {0.01, 0.02};
        public static double predict_vx = 0.0;
        public static double predict_vy = 0.0;
        public static double predict_x = 0.0;
        public static double predict_y = 0.0;
        public static double leadAngle = 0.0;
    }
}
