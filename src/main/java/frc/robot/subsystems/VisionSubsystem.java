package frc.robot.subsystems;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.Optional;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import com.ctre.phoenix6.swerve.SwerveDrivetrain.SwerveDriveState;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class VisionSubsystem extends SubsystemBase {

    @AutoLogOutput
    private boolean reefTargetIsRight = true;
    // private PhotonTrackedTarget closestReefTag = ;
    /* vision */
    public final boolean use_vision = false; // set to false to disable vision fusion and just use odometry
    public final boolean kuse_vision = false;
    public final Map<PhotonCamera, PhotonPoseEstimator> cameraEstimators = new HashMap<>();
    private final AprilTagFieldLayout aprilTagFieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);

    public static class VisionMeasurement {//融合位姿时需要的视觉数据
        public final Pose2d pose;
        public final double timestamp;
        public final Matrix<N3, N1> stdDevs;
    
        public VisionMeasurement(Pose2d pose, double timestamp, Matrix<N3, N1> stdDevs) {
            this.pose = pose;
            this.timestamp = timestamp;
            this.stdDevs = stdDevs;
        }
    }

    public VisionSubsystem(){

                // initialize camera system
        cameraEstimators.put(
            new PhotonCamera("Camera_up"),
            new PhotonPoseEstimator(
                aprilTagFieldLayout,
                PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
                new Transform3d(
                    new Translation3d(0.0, 0.0, 0.5),//z to elevator
                    new Rotation3d(     
                    0,
                    Units.degreesToRadians(0),//pitch
                    Units.degreesToRadians(0))//yaw
                )
            )
        );
        // cameraEstimators.put(
        //     new PhotonCamera("Camera_right"), 
        //     new PhotonPoseEstimator(
        //         aprilTagFieldLayout,
        //         PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
        //         new Transform3d(
        //             new Translation3d(0.31, -0.31, 0.5),
        //             new Rotation3d(     
        //                 0,
        //                 Units.degreesToRadians(0),//pitch
        //                 Units.degreesToRadians(0))//yaw
        //         )
        //     )
        // );
        System.out.println("finish camera initialization");

    }

    /**
     * Processes vision data from all registered cameras and updates the robot pose estimation.
     * This method:
     *   Publishes drivetrain odometry pose to the SmartDashboard.</li>
     *   Retrieves and processes the latest vision results from each camera.</li>
     *   Updates the pose estimator with vision measurements if vision is enabled.</li>
     *   Publishes vision pose, detected target IDs, and logs output for debugging.</li>
     * This should be called periodically (e.g., in {@code periodic()}) to keep vision-based localization up to date.
     */
    public List<VisionMeasurement> processVisionData(SwerveDriveState driveState) {
        List<VisionMeasurement> visionMeasurements = new ArrayList<>();

        Pose2d odometryPose = driveState.Pose;

        // report the pose of drivetrain
        SmartDashboard.putNumberArray("Chassis/Pose", 
            new double[] {
                odometryPose.getX(),
                odometryPose.getY(),
                odometryPose.getRotation().getDegrees()
            });

        cameraEstimators.forEach((camera, estimator) -> {
            List<PhotonPipelineResult> cameraResults = camera.getAllUnreadResults();
            if (cameraResults.isEmpty()) return;

            PhotonPipelineResult latestResult = cameraResults.get(cameraResults.size() - 1);
            if (!latestResult.hasTargets()) return;

            estimator.setReferencePose(odometryPose);

            Optional<EstimatedRobotPose> estimatedPose = estimator.update(latestResult);

            estimatedPose.ifPresent(pose -> {
                Matrix<N3, N1> stdDevs = calculateAdaptiveStdDevs(
                    latestResult.targets.size(),
                    calculateAverageDistance(latestResult.targets),
                    driveState.Speeds
                );

                Pose2d visionPose = pose.estimatedPose.toPose2d();
                double timestamp = latestResult.getTimestampSeconds();

                // 保存完整数据
                visionMeasurements.add(new VisionMeasurement(visionPose, timestamp, stdDevs));

                // report vision data
                String cameraName = camera.getName();
                SmartDashboard.putNumberArray("Vision/Pose/" + cameraName, 
                    new double[] {
                        visionPose.getX(),
                        visionPose.getY(),
                        visionPose.getRotation().getDegrees()
                    });

                // report ID
                int[] targetIds = latestResult.targets.stream()
                    .mapToInt(PhotonTrackedTarget::getFiducialId)
                    .toArray();
                SmartDashboard.putNumberArray("Vision/Targets/" + cameraName, 
                    Arrays.stream(targetIds).asDoubleStream().toArray());

                Logger.recordOutput("Vision/" + cameraName + "/Pose", pose.estimatedPose);
            });
        });

        return visionMeasurements;
    }
    
    
    /**
     * Calculates adaptive standard deviations for vision pose estimation
     * based on the number of AprilTags detected, their average distance,
     * and the current drivetrain speed.
     *
     * @param tagCount   Number of detected AprilTags.
     * @param avgDistance Average distance (in meters) from the camera to detected tags.
     * @param speeds     Current chassis speeds of the drivetrain.
     * @return A {@link Matrix} containing the standard deviations for X, Y, and rotation (θ).
     */
    private Matrix<N3, N1> calculateAdaptiveStdDevs(int tagCount, double avgDistance, ChassisSpeeds speeds) {
        double baseXY, baseTheta;
        
        // Set the base value based on the number of tags
        if (tagCount >= 2) {
            baseXY = 0.5;  
            baseTheta = Math.toRadians(5); 
        } else {
            baseXY = 1.5 + avgDistance * 0.2; 
            baseTheta = Math.toRadians(10 + avgDistance * 3); 
        }

        // Adjust dynamically based on speed
        double speedFactor = Math.hypot(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond) / 4.0; // suppose the speed max=4m/s
        double rotationFactor = Math.abs(speeds.omegaRadiansPerSecond) / Math.PI; // suppose the angular_speed=π rad/s
        
        return VecBuilder.fill(
            baseXY * (1 + speedFactor),       // X standard deviation
            baseXY * (1 + speedFactor),       // Y standard deviation
            baseTheta * (1 + rotationFactor)  // θ standard deviation
        );
    }

    /**
     * Calculates the average Euclidean distance from the camera to all detected AprilTags.
     *
     * @param targets List of detected {@link PhotonTrackedTarget} objects.
     * @return The average distance in meters, or 0.0 if no targets are present.
     */
    private double calculateAverageDistance(List<PhotonTrackedTarget> targets) {
        return targets.stream()
            .mapToDouble(t -> t.getBestCameraToTarget().getTranslation().getNorm())
            .average()
            .orElse(0.0);
    }


}

