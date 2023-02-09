package frc.robot.commands.Align;

<<<<<<< Updated upstream



import com.ctre.phoenix.motorcontrol.ControlMode;

=======
>>>>>>> Stashed changes
import org.photonvision.targeting.PhotonTrackedTarget;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.AlignConstants;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.FlareVisionSubsystem;

public class AlignForLeft extends CommandBase {
  public PhotonTrackedTarget target;
  private Boolean done = false;
<<<<<<< Updated upstream
  private int cone_number;
  
=======
>>>>>>> Stashed changes
  private double target_y;
  private double target_distance;
  private double gyro_aci;

  private double calc_y;
  private double cross_distance = -1;
  private double needed_angle;
<<<<<<< Updated upstream
  public AlignForLeft(DriveSubsystem driveSubsystem, int cone_number ) {
    this.cone_number = cone_number;
=======

  private double rot_gap = 10;
  private int index = 0;

  public AlignForLeft(DriveSubsystem driveSubsystem) {
>>>>>>> Stashed changes
    addRequirements(driveSubsystem);
  }

  @Override
  public void initialize() {
    if (FlareVisionSubsystem.getAprilTagID() != 8 && FlareVisionSubsystem.getAprilTagID() != 3)
      end(true);

    // System.out.println("AUTO Align Left Start");
    target = FlareVisionSubsystem.getBestTarget();
    gyro_aci = DriveSubsystem.m_gyro.getYaw();
    target_distance = FlareVisionSubsystem.getDistanceToGoal(target);
    target_y = FlareVisionSubsystem.getYdistance(target);
  }

  @Override
  public void execute() {
<<<<<<< Updated upstream
    
    if(FlareVisionSubsystem.getAprilTagID() == 8 || FlareVisionSubsystem.getAprilTagID() == 3){
      if(cross_distance <=0){
      calc_y = AlignConstants.cone_1_distance + target_y;
      cross_distance = Math.sqrt(Math.pow(calc_y,2)+Math.pow(target_distance,2))-10;
      needed_angle = Math.acos(target_distance/cross_distance);
    }else if(!DriveSubsystem.turn_angles(needed_angle, gyro_aci)){
      DriveSubsystem.
    }
      
    

      



      
=======
    if (cross_distance <= 0) {
      calc_y = AlignConstants.outermost_cone_distance + target_y;
      cross_distance = Math.sqrt(Math.pow(Math.abs(calc_y), 2) + Math.pow(target_distance, 2)) - rot_gap;
      needed_angle = Math.acos(target_distance / cross_distance);
>>>>>>> Stashed changes
    }

    if (index == 0) {
      index = 1;
      gyro_aci = DriveSubsystem.m_gyro.getYaw();
    }

    if (!DriveSubsystem.turn_angles(needed_angle, gyro_aci, Math.abs(target_y) > AlignConstants.outermost_cone_distance))
      return;

    if (!DriveSubsystem.drive_PID_centimeters(cross_distance))
      return;

    if (index == 1) {
      index = 2;
      gyro_aci = DriveSubsystem.m_gyro.getYaw();
    }

    if (!DriveSubsystem.turn_angles(gyro_aci, needed_angle, !(Math.abs(target_y) > AlignConstants.outermost_cone_distance)))
      return;

    if (DriveSubsystem.drive_PID_centimeters(rot_gap))
      done = true;
  }

  @Override
  public void end(boolean interrupted) {
<<<<<<< Updated upstream
  
    // System.out.println("AUTO Align LEFT End");
=======
    // System.out.println("AUTO Align Left End");
>>>>>>> Stashed changes
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
