package frc.robot.subsystems;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.FlrvsnConstants;



public class FlareVisionSubsystem extends SubsystemBase {


  public static  PhotonCamera camera;
 

  public FlareVisionSubsystem() {
   
  }
  @Override
  public void periodic() {
    camera = new PhotonCamera("Camera");
}

public static PhotonTrackedTarget getBestTarget() {
  PhotonPipelineResult result = camera.getLatestResult();

  if (result == null) return null;
  
  if (result.hasTargets()) {
    return result.getBestTarget();
  } else {
    return null;
  }
}
/*public static double getYaw() {
  // Get the best target
  if (getBestTarget() != null) {
    return getBestTarget().getYaw();
  } else {
    return 0;
  }
}
*/
public static int getAprilTagID(){
  var target = getBestTarget();
  return target.getFiducialId();
}


public static double getDistanceToGoal(PhotonTrackedTarget besTarget){
  return besTarget.getBestCameraToTarget().getX();
}

public static double getYdistance(PhotonTrackedTarget besTarget){
  return besTarget.getBestCameraToTarget().getY();
}

}
/*public static double getDistanceToGoal(PhotonTrackedTarget bestTarget) {
  if (bestTarget != null) {
    double angleToGoalDegrees =
      Constants.FlrvsnConstants.Camera_Pitch + bestTarget.getPitch();
    double angleToGoalRadians = angleToGoalDegrees * (3.14159 / 180.0);
    return (
      (
        Constants.FlrvsnConstants.TARGET_HEIGHT_METERS -
        Constants.FlrvsnConstants.CAMERA_HEIGHT_METERS
      ) /
      Math.tan(angleToGoalRadians) *
      4.75d
    );
  } else {
    return 0;
  }
}
*/