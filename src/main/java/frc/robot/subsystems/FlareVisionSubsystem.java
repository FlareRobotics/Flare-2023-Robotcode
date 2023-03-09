package frc.robot.subsystems;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class FlareVisionSubsystem extends SubsystemBase {

  public static PhotonCamera camera;

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