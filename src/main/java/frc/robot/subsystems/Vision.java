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

    @AutoLogOutput
    private boolean reefTargetIsRight = true;
    // private PhotonTrackedTarget closestReefTag = ;
    public final boolean use_vision = false; // set to false to disable vision fusion and just use odometry
    public final boolean kuse_vision = false;
    public final Map<PhotonCamera, PhotonPoseEstimator> cameraEstimators = new HashMap<>();
    private final AprilTagFieldLayout aprilTagFieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);

    /**
    融合位姿时需要的视觉数据
     */
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

    public Vision(){
        // cameraEstimators.put(
        //     new PhotonCamera("Camera_Up"),
        //     new PhotonPoseEstimator(
        //         aprilTagFieldLayout,
        //         PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
        //         new Transform3d(
        //             new Translation3d(0.0, 0.0, 0.5),//z to elevator
        //             new Rotation3d(     
        //             0,
        //             Units.degreesToRadians(0),//pitch
        //             Units.degreesToRadians(0))//yaw
        //         )
        //     )
        // );

        cameraEstimators.put(
            new PhotonCamera("Camera_FL"),
            new PhotonPoseEstimator(
                aprilTagFieldLayout,
                PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
                new Transform3d(
                    new Translation3d(-0.01578, 0.16625, 0.53436),//z to elevator
                    new Rotation3d(     
                    0,
                    Units.degreesToRadians(0),//pitch
                    Units.degreesToRadians(0))//yaw
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
                    new Rotation3d(     
                        0,
                        Units.degreesToRadians(0),//pitch
                        Units.degreesToRadians(0))//yaw
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
                    new Rotation3d(     
                        0,
                        Units.degreesToRadians(0),//pitch
                        Units.degreesToRadians(180))//yaw
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
                    new Rotation3d(     
                        0,
                        Units.degreesToRadians(0),//pitch
                        Units.degreesToRadians(180))//yaw
                )
            )
        );
        System.out.println("Camera Init Finished");
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

            // // 单标签 Ambiguity (歧义) 过滤,解决 MULTI_TAG 模式下退化为单标签时引发的位姿跳变问题
            // if (latestResult.targets.size() == 1) {
            //     PhotonTrackedTarget target = latestResult.targets.get(0);
            //     double ambiguity = target.getPoseAmbiguity();
                
            //     // 如果只有一个标签，且模糊度过高(>0.2)或处于画面边缘(-1.0)，
            //     // 说明极有可能发生了 PnP 翻转错觉，直接丢弃该相机当前帧的数据
            //     if (ambiguity > 0.2 || ambiguity == -1.0) {
            //         return; // 退出当前 lambda，跳过此相机的更新
            //     }
            // }

            // 提取歧义度，如果是多标签则设为 0
            // ==========================================
            final double ambiguity; // 声明为 final，确保只被赋值一次，解决 Lambda 报错
            
            if (latestResult.targets.size() == 1) {
                PhotonTrackedTarget target = latestResult.targets.get(0);
                ambiguity = target.getPoseAmbiguity();
                
                // 保留这层最基础的安全网：防翻转跳变
                if (ambiguity > 0.2 || ambiguity == -1.0) {
                    return; // 歧义过大直接丢弃当前帧
                }
            } else {
                ambiguity = 0.0; // 多标签时，一次性赋值为 0
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

                // 传入 tag数量、距离、速度、和单标签的歧义度
                // 此时使用 ambiguity 就绝对不会报错了！
                Matrix<N3, N1> stdDevs = calculateAdaptiveStdDevs(
                    tagCount,
                    avgDistance,
                    driveState.Speeds,
                    ambiguity
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

     * 根据标签数量、平均距离和底盘速度，动态计算视觉置信度(标准差)。
     * 距离越远，标准差呈指数级飙升，底盘会自动降低对视觉的信任，避免漂移。
     */
    private Matrix<N3, N1> calculateAdaptiveStdDevs(int tagCount, double avgDistance, ChassisSpeeds speeds, double ambiguity) {
        double baseXY, baseTheta;
        
        // 【核心改进】：距离敏感度惩罚项，使用距离的平方！
        // 距离 1米 -> 乘数 1; 距离 3米 -> 乘数 9; 距离 5米 -> 乘数 25!
        double distanceMultiplier = Math.pow(avgDistance, 2); 
        
        if (tagCount >= 2) {
            // 多标签模式：基础精度高，但也受距离影响
            // 举例：1m处误差为 0.15, 3m处误差为 0.55, 5m处误差飙升到 1.35

            //此参数更信任视觉如想要更精准可以改回来
            // baseXY = 0.1 + (0.05 * distanceMultiplier);  
            // baseTheta = Math.toRadians(2 + (0.5 * distanceMultiplier)); 

            baseXY = 0.5 + (0.15 * distanceMultiplier);  
            baseTheta = Math.toRadians(5 + (2.0 * distanceMultiplier)); 
        } else {
            // 单标签模式：基础精度低，远距离误差极其离谱
            // 举例：1m处误差为 0.65, 3m处误差为 1.85, 5m处误差飙升到 4.25 (底盘几乎不再信任)
            baseXY = 0.5 + (0.15 * distanceMultiplier); 
            baseTheta = Math.toRadians(5 + (2.0 * distanceMultiplier)); 

            // 如果有歧义(Ambiguity)，额外惩罚
            if (ambiguity > 0) {
                baseXY *= (1.0 + (ambiguity * 2.0));
                baseTheta *= (1.0 + (ambiguity * 2.0));
            }
        }

        // 基于运动速度的惩罚保持不变（运动越快，快门模糊导致误差越大）
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

