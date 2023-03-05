package frc.robot.commands.Align;

import org.photonvision.targeting.PhotonTrackedTarget;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.AlignConstants;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.FlareVisionSubsystem;

public class AlignForLeft_2 extends CommandBase {
  public PhotonTrackedTarget target;
  private Boolean done = false;
  private double target_y;
  private double target_distance;
  private double gyro_aci;

  private double calc_y;
  private double cross_distance = -1;
  private double needed_angle;

  private double rot_gap = 10;
  private int index = 0;

  public AlignForLeft_2(DriveSubsystem driveSubsystem) {
    addRequirements(driveSubsystem);
  }

  @Override
  public void initialize() {
    if (FlareVisionSubsystem.getAprilTagID() != 8 && FlareVisionSubsystem.getAprilTagID() != 3)
      end(true);

    // System.out.println("AUTO Align Left_2 Start");
    target = FlareVisionSubsystem.getBestTarget();
    gyro_aci = DriveSubsystem.m_gyro.getYaw();
    target_distance = FlareVisionSubsystem.getDistanceToGoal(target);
    target_y = FlareVisionSubsystem.getYdistance(target);
  }

  @Override
  public void execute() {
    if (cross_distance <= 0) {
      calc_y = AlignConstants.cone_distance - target_y;
      cross_distance = Math.sqrt(Math.pow(Math.abs(calc_y), 2) + Math.pow(target_distance, 2)) - rot_gap;
      needed_angle = Math.acos(target_distance / cross_distance);
    }

    if (index == 0) {
      index = 1;
      gyro_aci = DriveSubsystem.m_gyro.getYaw();
    }

    if (!DriveSubsystem.turn_angles(needed_angle, gyro_aci, true))
      return;

    if (!DriveSubsystem.drive_PID_centimeters(cross_distance))
      return;

    if (index == 1) {
      index = 2;
      gyro_aci = DriveSubsystem.m_gyro.getYaw();
    }

    if (!DriveSubsystem.turn_angles(gyro_aci, needed_angle, false))
      return;

    if (DriveSubsystem.drive_PID_centimeters(rot_gap))
      done = true;
  }

  @Override
  public void end(boolean interrupted) {
    // System.out.println("AUTO Align Left_2 End");
  }

  @Override
  public boolean isFinished() {
    return done;
  }
}
