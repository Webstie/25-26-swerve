package frc.robot;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.wpilibj.XboxController;

public class Constants {

    /** LED/background states used by CANdleSystem. */
    public static class RobotState {
        public enum State {
            Shooting, Intaking, ClimbingUp, Outtaking, ClimbingDown, Idle, VisionFusion;
        }
    }

    /** Field dimensions in meters. Used for alliance mirroring and hub-relative checks. */
    public static class Layout {
        public static final double FIELD_LENGTH_METERS = 16.540988;
        public static final double FIELD_WIDTH_METERS = 8.0692752;
    }

    /** CANdle IDs plus legacy test buttons/brightness positions. */
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

    /** Climber motor ID and Motion Magic target positions in motor rotations. */
    public static final class ClimberConfig {
        public static final double ClimberTopPosition = 95.0;
        public static final double ClimbPosition = 1.0;
        public static final int CLIMBER_MOTOR_ID = 1;
    }

    /** Shooter, feeder, transport-to-shooter, and launcher pitch tuning. */
    public static final class LauncherConfig {
        // Shooter flywheel speeds are in rotations per second (rps).
        public static final double WarmupSpeed = 60.0;
        // Default warmup wait/timeout before feeding balls into the shooter.
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

    /** Intake roller and pitch-arm tuning. Pitch positions are motor rotations. */
    public static final class IntakeConfig {
        // Swing positions used while shooting/feeding to keep the ball moving.
        public static final double IntakeSwingDownPosition = -16.0;
        public static final double IntakeSwingUpPosition = -13.0;
        // Stowed/deployed intake pitch positions.
        public static final double IntakeUpPosition = 0.0;
        public static final double IntakeDownPosition = -17.5;
        // Delay at each swing endpoint. Smaller = faster shake frequency.
        public static final double SwingWaitTime = 0.1;
        // Motion Magic limits used only while shaking/swinging intake during shots.
        public static final double IntakeSwingUpAcceleration = 300.0;
        public static final double IntakeSwingUpCruiseVelocity = 400.0;
        public static final double IntakeSwingDownAcceleration = 300.0;
        public static final double IntakeSwingDownCruiseVelocity = 400.0;
        // Roller velocities are in rotations per second (rps). Sign controls direction.
        public static final double OuttakeVelocity = 20.0;
        public static final double IntakeVelocity = -90.0;
        // Dynamic Motion Magic limits for intake pitch. Up/down are selected by target direction.
        public static final double IntakePitchUpAcceleration = 300.0;
        public static final double IntakePitchUpCruiseVelocity = 200.0;
        public static final double IntakePitchDownAcceleration = 300.0;
        public static final double IntakePitchDownCruiseVelocity = 300.0;

        public static final int INTAKE_LEFT_MOTOR_ID = 6;
        public static final int INTAKE_RIGHT_MOTOR_ID = 7;
        public static final int INTAKE_PITCH_MOTOR_ID = 9;
    }

    /** Ball transport between intake and shooter. Velocity is rps. */
    public static final class TransportConfig {
        public static final double TransportSpeed = 90.0;
        public static final int TRANSPORT_MOTOR_ID = 10;
    }

    /** Teleop drive scaling. Aim scales are used while driver holds auto-aim shooting. */
    public static final class DriveConfig {
        public static final double TeleopDriveSpeedScale = 0.75;
        public static final double AimDriveScaleX = 0.5;
        public static final double AimDriveScaleY = 0.4;
    }

    /** Field geometry, AprilTag layout, and all vision-assisted shooting lookup tables. */
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
        // Applied to the speed column below before in-match ShootingTrim.speedOffset.
        public static final double POINTS_SPEED_OFFSET = -2.0;

        public static final double[][] POINTS_PARAMS_TABLE_BLUE = {
            {3.2,   5.4,   -0.002, 52},   // Point 1 (near left)
            {2.8,   4.03,  -0.002, 52},   // Point 2 (near mid)
            {3.2,   2.6,   -0.002, 52},   // Point 3 (near right)
            {2.17,  6.01,  -0.009,  59}, // Point 4 (far left)
            {1.642, 3.61,  -0.009,  59}, // Point 5 (far mid)
            {2.039, 2.065, -0.009,  59}, // Point 6 (far right)
        };

        // Distance-based interpolation table for dynamic shooting
        // Columns: {distance_m, pitch_rot, speed_rps, boost_rps}
        // Applied to both speed columns below before they enter the interpolation maps.
        // Positive = faster shooter, negative = slower shooter.
        public static final double DISTANCE_SPEED_OFFSET = -2.0;

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
            //{1.25, -0.0000,  45.00,  47.00},  // Point 1
            {1.50, -0.0000,  48.50,  49.00},    // Point 2
            //{1.75, -0.0000,  49.50,  51.00},    // Point 3
            {2.00, -0.0015,  52.50,  55.00},    // Point 4
            //{2.25, -0.0020,  52.00,  53.50},    // Point 5
            {2.50, -0.0040,  55.50,  57.25},    // Point 6
            //{2.75, -0.0055,  55.25,  56.50},    // Point 7
            {3.00, -0.0065,  57.50,  59.25},    // Point 8
            //{3.25, -0.0090,  59.00,  60.00},    // Point 9
            {3.50, -0.0100,  61.00,  63.50},    // Point 10
            //{3.75, -0.0105,  60.25,  61.00},    // Point 11
            {4.00, -0.0120,  62.50,  64.50},    // Point 12
            //{4.25, -0.0125,  61.75,  63.00},    // Point 13
            {4.50, -0.0140,  64.00,  68.00},    // Point 14
            //{4.75, -0.0170,  64.00,  65.50},    // Point 15
            {5.00, -0.0180,  67.00,  69.50},    // Point 16
            //{5.25, -0.0190,  66.00,  68.50},    // Point 17
            {5.50, -0.0200,  68.50,  72.00},    // Point 18
        };

        public static final InterpolatingDoubleTreeMap distanceToPitchMap = new InterpolatingDoubleTreeMap();
        public static final InterpolatingDoubleTreeMap distanceToSpeedMap = new InterpolatingDoubleTreeMap();
        public static final InterpolatingDoubleTreeMap distanceToBoostSpeedMap = new InterpolatingDoubleTreeMap();

        // Build interpolation maps once at startup from DISTANCE_PARAMS_TABLE.
        static {
            for (double[] point : DISTANCE_PARAMS_TABLE) {
                distanceToPitchMap.put(point[0], point[1]);
                distanceToSpeedMap.put(point[0], point[2] + DISTANCE_SPEED_OFFSET);
                distanceToBoostSpeedMap.put(point[0], point[3] + DISTANCE_SPEED_OFFSET);
            }
        }

        // Corner feed targets in the blue alliance frame.
        public static final Translation2d BLUE_CORNER_LEFT  = new Translation2d(1.5, 7.0);
        public static final Translation2d BLUE_CORNER_RIGHT = new Translation2d(1.5, 1.0);

        // Reject corner feed if robot is closer than this to the hub (ball won't clear)
        public static final double CORNER_FEED_MIN_HUB_DISTANCE = 0.5;

        // Corner feed distance -> {pitch_rot, speed_rps}; used for long lob shots.
        public static final double[][] CORNER_FEED_DISTANCE_TABLE = {
            {5.0,  -0.015,  70.0},
            {6.0,  -0.020,  75.0},
            {7.0,  -0.025,  80.0},
            {8.0, -0.030,  85.0},
        };

        public static final InterpolatingDoubleTreeMap distanceToCornerPitchMap = new InterpolatingDoubleTreeMap();
        public static final InterpolatingDoubleTreeMap distanceToCornerSpeedMap = new InterpolatingDoubleTreeMap();

        // Build interpolation maps once at startup from CORNER_FEED_DISTANCE_TABLE.
        static {
            for (double[] point : CORNER_FEED_DISTANCE_TABLE) {
                distanceToCornerPitchMap.put(point[0], point[1]);
                distanceToCornerSpeedMap.put(point[0], point[2]);
            }
        }

        // Vision lineup tolerances used by drivetrain/aim commands.
        public static final double LINEUP_TOLERANCE_METERS = 0.015;
        public static final double ANGLE_TOLERANCE_DEGREES = 3.0;
    }

    /** Global in-match shooting trim: additive offsets applied on top of all table lookups. */
    public static class ShootingTrim {
        // Operator bumpers/triggers adjust by these amounts each press.
        public static final double SPEED_TRIM_STEP = 1.25;
        public static final double PITCH_TRIM_STEP = 0.001;
        /** Speed offset (rps): positive = faster */
        public static double speedOffset = 0.0;
        /** Pitch offset (rotations): positive = higher angle */
        public static double pitchOffset = 0.0;
    }

    /** Tunables/state for the simple lead prediction filter used while aiming. */
    public static class KalmanFilterConfig {
        public static final double kDt = 0.020; // 20ms
        // Process and measurement noise terms for the filter state.
        public static final double[] stateStdDevs = {0.01, 0.1, 0.5};
        public static final double[] measurementStdDevs = {0.01, 0.02};
        // Predicted target/robot-relative state used by aim compensation.
        public static double predict_vx = 0.0;
        public static double predict_vy = 0.0;
        public static double predict_x = 0.0;
        public static double predict_y = 0.0;
        public static double leadAngle = 0.0;
    }
}
