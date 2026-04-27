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

public class Vision extends SubsystemBase {

    public final Map<PhotonCamera, PhotonPoseEstimator> cameraEstimators = new HashMap<>();
    private final AprilTagFieldLayout aprilTagFieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);

    public static class VisionMeasurement {
        public final Pose2d pose;
        public final double timestamp;
        public final Matrix<N3, N1> stdDevs;

        public VisionMeasurement(Pose2d pose, double timestamp, Matrix<N3, N1> stdDevs) {
            this.pose = pose;
            this.timestamp = timestamp;
            this.stdDevs = stdDevs;
        }
    }

    public Vision() {
        cameraEstimators.put(
            new PhotonCamera("Camera_FL"),
            new PhotonPoseEstimator(
                aprilTagFieldLayout,
                PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
                new Transform3d(
                    new Translation3d(-0.01578, 0.16625, 0.53436),
                    new Rotation3d(0, Units.degreesToRadians(0), Units.degreesToRadians(0))
                )
            )
        );

        cameraEstimators.put(
            new PhotonCamera("Camera_FR"),
            new PhotonPoseEstimator(
                aprilTagFieldLayout,
                PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
                new Transform3d(
                    new Translation3d(-0.01578, -0.16625, 0.53436),
                    new Rotation3d(0, Units.degreesToRadians(0), Units.degreesToRadians(0))
                )
            )
        );

        cameraEstimators.put(
            new PhotonCamera("Camera_BL"),
            new PhotonPoseEstimator(
                aprilTagFieldLayout,
                PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
                new Transform3d(
                    new Translation3d(-0.32704, 0.2765, 0.5),
                    new Rotation3d(0, Units.degreesToRadians(0), Units.degreesToRadians(180))
                )
            )
        );

        cameraEstimators.put(
            new PhotonCamera("Camera_BR"),
            new PhotonPoseEstimator(
                aprilTagFieldLayout,
                PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
                new Transform3d(
                    new Translation3d(-0.32704, -0.2765, 0.5),
                    new Rotation3d(0, Units.degreesToRadians(0), Units.degreesToRadians(180))
                )
            )
        );

        System.out.println("Camera Init Finished");
    }

    /**
     * Processes vision data from all registered cameras and returns pose measurements
     * for odometry fusion. Publishes chassis and vision poses to SmartDashboard.
     */
    public List<VisionMeasurement> processVisionData(SwerveDriveState driveState) {
        List<VisionMeasurement> visionMeasurements = new ArrayList<>();

        Pose2d odometryPose = driveState.Pose;

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

            final double ambiguity;
            if (latestResult.targets.size() == 1) {
                PhotonTrackedTarget target = latestResult.targets.get(0);
                ambiguity = target.getPoseAmbiguity();
                if (ambiguity > 0.2 || ambiguity == -1.0) {
                    return;
                }
            } else {
                ambiguity = 0.0;
            }

            estimator.setReferencePose(odometryPose);
            Optional<EstimatedRobotPose> estimatedPose = estimator.update(latestResult);

            estimatedPose.ifPresent(pose -> {
                double zHeight = pose.estimatedPose.getZ();
                if (Math.abs(zHeight) > 0.5) {
                    return;
                }

                int tagCount = latestResult.targets.size();
                double avgDistance = calculateAverageDistance(latestResult.targets);

                Matrix<N3, N1> stdDevs = calculateAdaptiveStdDevs(
                    tagCount, avgDistance, driveState.Speeds, ambiguity);

                Pose2d visionPose = pose.estimatedPose.toPose2d();
                double timestamp = latestResult.getTimestampSeconds();

                visionMeasurements.add(new VisionMeasurement(visionPose, timestamp, stdDevs));

                String cameraName = camera.getName();
                SmartDashboard.putNumberArray("Vision/Pose/" + cameraName,
                    new double[] {
                        visionPose.getX(),
                        visionPose.getY(),
                        visionPose.getRotation().getDegrees()
                    });

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
     * Calculates adaptive standard deviations for vision pose estimation based on
     * tag count, average distance, chassis speed, and single-tag ambiguity.
     * Distance penalty grows quadratically; higher speed further degrades trust.
     */
    private Matrix<N3, N1> calculateAdaptiveStdDevs(int tagCount, double avgDistance,
                                                      ChassisSpeeds speeds, double ambiguity) {
        double distanceMultiplier = Math.pow(avgDistance, 2);
        double baseXY = 0.5 + (0.15 * distanceMultiplier);
        double baseTheta = Math.toRadians(5 + (2.0 * distanceMultiplier));

        if (tagCount == 1 && ambiguity > 0) {
            baseXY *= (1.0 + (ambiguity * 2.0));
            baseTheta *= (1.0 + (ambiguity * 2.0));
        }

        double speedFactor = Math.hypot(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond) / 4.0;
        double rotationFactor = Math.abs(speeds.omegaRadiansPerSecond) / Math.PI;

        return VecBuilder.fill(
            baseXY * (1 + speedFactor),
            baseXY * (1 + speedFactor),
            baseTheta * (1 + rotationFactor)
        );
    }

    /**
     * Calculates the average Euclidean distance from the camera to all detected AprilTags.
     */
    private double calculateAverageDistance(List<PhotonTrackedTarget> targets) {
        return targets.stream()
            .mapToDouble(t -> t.getBestCameraToTarget().getTranslation().getNorm())
            .average()
            .orElse(0.0);
    }
}
