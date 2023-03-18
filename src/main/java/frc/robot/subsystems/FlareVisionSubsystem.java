package frc.robot.subsystems;

import java.io.IOException;
import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class FlareVisionSubsystem extends SubsystemBase {

  public static PhotonCamera camera;
  private static PhotonPoseEstimator photonPoseEstimator;

  public FlareVisionSubsystem()
  {
    camera = new PhotonCamera("Camera");
    try {
      // Attempt to load the AprilTagFieldLayout that will tell us where the tags are on the field.
      AprilTagFieldLayout fieldLayout = AprilTagFields.k2023ChargedUp.loadAprilTagLayoutField();
      // Create pose estimator
      photonPoseEstimator =
              new PhotonPoseEstimator(
                      fieldLayout, PoseStrategy.AVERAGE_BEST_TARGETS, camera, new Transform3d());
      photonPoseEstimator.setStrategy(PoseStrategy.LOWEST_AMBIGUITY);
  } catch (IOException e) {
      // The AprilTagFieldLayout failed to load. We won't be able to estimate poses if we don't know
      // where the tags are.
      photonPoseEstimator = null;
  }
  }

  public static Optional<EstimatedRobotPose> getEstimatedGlobalPose(Pose2d prevEstimatedRobotPose) {
    if (photonPoseEstimator == null) {
        // The field layout failed to load, so we cannot estimate poses.
        return Optional.empty();
    }
    photonPoseEstimator.setReferencePose(prevEstimatedRobotPose);
    return photonPoseEstimator.update();
}
  @Override
  public void periodic() {
    camera = new PhotonCamera("Camera");
    SmartDashboard.putNumber("CurrentAprilTag", getAprilTagID());
    SmartDashboard.putNumber("XDistanceToTag", getDistanceToGoal(getBestTarget()));
    SmartDashboard.putNumber("YDistanceToTag", getYdistance(getBestTarget()));
  }

  public static PhotonTrackedTarget getBestTarget() {
    PhotonPipelineResult result = camera.getLatestResult();

    if (result == null)
      return null;

    if (result.hasTargets()) {
      return result.getBestTarget();
    } else {
      return null;
    }
  }

  public static int getAprilTagID() {
    var target = getBestTarget();
    return target == null ? -1 : target.getFiducialId();
  }

  public static double getDistanceToGoal(PhotonTrackedTarget besTarget) {
    return besTarget == null ? 0 : besTarget.getBestCameraToTarget().getX();
  }

  public static double getYdistance(PhotonTrackedTarget besTarget) {
    return besTarget == null ? 0 : besTarget.getBestCameraToTarget().getY();
  }
}