package frc.robot.subsystems;

import java.util.List;
import java.util.Optional;
import java.util.Collection;
import java.util.Collections;

import org.littletonrobotics.junction.Logger;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import com.ctre.phoenix6.hardware.Pigeon2;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;

public class VisionSub extends SubsystemBase {

    private final PhotonCamera camera;
    public PhotonTrackedTarget currentTarget;
    public PhotonPipelineResult latestResult;

    public Pigeon2 pigeon2 = new Pigeon2(14);
    private PhotonPoseEstimator photonPoseEstimator;
    PhotonPipelineResult result = new PhotonPipelineResult();

    private AprilTagFieldLayout fieldLayout;

    private final Transform3d robotToCam = new Transform3d(
            new Translation3d(0.0381, 0.0, 0.47), // x:forward, y:left, z:up This is how far away in meters the camera
                                                    // is from the pigeon
            new Rotation3d(0.0, Math.toRadians(-16.1), 0.0) // this is for the backcam
    );

    public InterpolatingDoubleTreeMap hdssm;
    public InterpolatingDoubleTreeMap hdftm;

    // private static final Pose3d HUB_BLUE = new Pose3d(
    // 4.612,
    // 4.021,
    // 0.0,
    // new Rotation3d());

    private final CommandSwerveDrivetrain m_drivetrain;

    public VisionSub(CommandSwerveDrivetrain drivetrain, PhotonCamera frontcam) {
        camera = frontcam;
        m_drivetrain = drivetrain;

    }

    public void setAprilTagField(AprilTagFieldLayout atfl) {
        this.fieldLayout = atfl;

        photonPoseEstimator = new PhotonPoseEstimator(
                fieldLayout,
                PhotonPoseEstimator.PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
                robotToCam);

        photonPoseEstimator.setMultiTagFallbackStrategy(
                PhotonPoseEstimator.PoseStrategy.LOWEST_AMBIGUITY);
    }

    @Override
    public void periodic() {
        var results = camera.getAllUnreadResults();

        if (results.isEmpty())
            return;

        // Always use the most recent frame
        var result = results.get(results.size() - 1);

        if (!result.hasTargets())
            return;

        // Get estimated robot pose from PhotonVision
        var estimatedPoseOpt = photonPoseEstimator.update(result);

        if (estimatedPoseOpt.isEmpty())
            return;

        var estimatedPose = estimatedPoseOpt.get();

        Pose2d visionPose = estimatedPose.estimatedPose.toPose2d();
        double timestamp = estimatedPose.timestampSeconds;

        // 🔎 Optional: reject bad measurements
        var bestTarget = result.getBestTarget();
        boolean isGoodMeasurement = result.getTargets().size() >= 2 ||
                bestTarget.getPoseAmbiguity() < 0.2;

        if (!isGoodMeasurement)
            return;

        // 🚀 THIS is the important line
        m_drivetrain.addVisionMeasurement(visionPose, timestamp);

        // (Optional) Logging for AdvantageScope
        Logger.recordOutput("Vision/Pose", visionPose);
        Logger.recordOutput("Vision/Ambiguity", bestTarget.getPoseAmbiguity());
    }

    public boolean hasTarget() {
        return currentTarget != null;
    }

    // get the fore and aft distance to the target
    public double getTargetX() {
        if (currentTarget != null && currentTarget.bestCameraToTarget != null) {
            return currentTarget.bestCameraToTarget.getX();
        }
        return 0.0;
    }

    // get the left and right distance to cam center
    public double getTargetY() {
        if (currentTarget != null && currentTarget.bestCameraToTarget != null) {
            return currentTarget.bestCameraToTarget.getY();
        }
        return 0.0;
    }

    public double getYaw() {
        if (currentTarget != null && currentTarget.bestCameraToTarget != null) {
            var result = latestResult;
            double targetYaw = result.getBestTarget().getYaw();
            return targetYaw;
        }
        return 0.0;
    }

    public double getDistanceToHub(double velocity_x, double velocity_y) {
        double distance = 0.0;

        // Hub position (Blue side)
        double HUB_X = 4.612;
        double HUB_Y = 4.021;

        double fieldLength = fieldLayout.getFieldLength();

        if (DriverStation.getAlliance().isPresent() &&
                DriverStation.getAlliance().get() == Alliance.Red) {
            HUB_X = fieldLength - HUB_X;
            velocity_x *= -1;
            velocity_y *= -1;
        }

        boolean targetVisible = false;
        var result = latestResult;

        if (result.hasTargets()) {
            var target = result.getBestTarget();
            Optional<Pose3d> tagPoseOptional = fieldLayout.getTagPose(target.getFiducialId());

            if (tagPoseOptional.isPresent()) {
                Pose3d tagPose = tagPoseOptional.get();
                Transform3d camToTag = target.getBestCameraToTarget();
                Transform3d robotToCamera = new Transform3d(
                        new Translation3d(0.0508, 0.0, 0.4318),
                        new Rotation3d(0, Math.toRadians(16.1), 0));

                Pose3d cameraPose = tagPose.transformBy(camToTag.inverse());
                Pose3d robotPose = cameraPose.transformBy(robotToCamera.inverse());

                double robotX = robotPose.getX();
                double robotY = robotPose.getY();

                // Standard Euclidean distance
                double dx = HUB_X - robotX;
                double dy = HUB_Y - robotY;
                double rawDistance = Math.hypot(dx, dy);

                // --- Compensate for motion ---
                double speed = Math.hypot(velocity_x, velocity_y);
                double flightTime = hdftm.get(rawDistance); // in seconds
                double adjustedX = HUB_X - velocity_x * flightTime;
                double adjustedY = HUB_Y - velocity_y * flightTime;

                distance = Math.hypot(adjustedX - robotX, adjustedY - robotY);

                targetVisible = true;
            }
        }

        if (!targetVisible) {
            distance = 0.0;
        }

        return distance;
    }

    public double getRotationToHub(CommandSwerveDrivetrain drivetrain,
            double velocity_x,
            double velocity_y,
            InterpolatingDoubleTreeMap hdftm) {
        double turn = 0.0;

        // Hub position (Blue side only)
        double HUB_X = 4.612;
        double HUB_Y = 4.021;

        double fieldLength = fieldLayout.getFieldLength();

        if (DriverStation.getAlliance().isPresent() &&
                DriverStation.getAlliance().get() == Alliance.Red) {

            HUB_X = fieldLength - HUB_X;
            velocity_x *= -1;
            velocity_y *= -1;
        }

        Transform3d robotToCamera = new Transform3d(
                new Translation3d(0.0508, 0.0, 0.4318),
                new Rotation3d(0, Math.toRadians(16.1), 0));

        boolean targetVisible = false;
        var result = latestResult;

        if (result.hasTargets()) {
            var target = result.getBestTarget();
            int fiducialId = target.getFiducialId();
            Optional<Pose3d> tagPoseOptional = fieldLayout.getTagPose(fiducialId);

            if (tagPoseOptional.isPresent()) {
                Pose3d tagPose = tagPoseOptional.get();
                Transform3d camToTag = target.getBestCameraToTarget();
                Pose3d cameraPose = tagPose.transformBy(camToTag.inverse());
                Pose3d robotPose = cameraPose.transformBy(robotToCamera.inverse());

                double robotX = robotPose.getX();
                double robotY = robotPose.getY();

                // System.out.println("Robot Pose: (" + robotX + ", " + robotY + ")");

                // Vector to hub
                double dx = HUB_X - robotX;
                double dy = HUB_Y - robotY;
                double distance = Math.hypot(dx, dy);

                // --- Shoot-on-the-move compensation ---

                double adjustedHubX = HUB_X;
                double adjustedHubY = HUB_Y;

                double speed = Math.hypot(velocity_x, velocity_y);
                if (speed > 0.1) { // only compensate if actually moving
                    double flightTime = hdftm.get(distance);
                    adjustedHubX = HUB_X - velocity_x * flightTime;
                    adjustedHubY = HUB_Y - velocity_y * flightTime;

                    // System.out.println(
                    // "FlightTime: " + flightTime + " | AdjHub: (" + adjustedHubX + ", " +
                    // adjustedHubY + ")");
                }

                // Desired heading to the (potentially adjusted) hub
                double adjDx = adjustedHubX - robotX;
                double adjDy = adjustedHubY - robotY;
                double desiredAngle = Math.atan2(adjDy, adjDx);

                // Current robot heading
                double currentAngle = drivetrain.getPose().getRotation().getRadians();

                // Normalized angle error
                double error = desiredAngle - currentAngle;
                error = Math.atan2(Math.sin(error), Math.cos(error));

                double VISION_TURN_kP = 1.0;
                turn = error * VISION_TURN_kP;

                // System.out.println("Distance: " + distance + " | Angle Error: " + error + " |
                // Turn: " + turn);

                targetVisible = true;
            }
        }

        return turn;
    }

    public Optional<PhotonTrackedTarget> getTarget() {
        return Optional.ofNullable(currentTarget);
    }
}
