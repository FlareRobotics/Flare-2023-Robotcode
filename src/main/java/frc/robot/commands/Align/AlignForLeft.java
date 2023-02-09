// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Align;




import com.ctre.phoenix.motorcontrol.ControlMode;

import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.AlignConstants;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.FlareVisionSubsystem;

/** An example command that uses an example subsystem. */

public class AlignForLeft extends CommandBase {
  public PhotonTrackedTarget target;
  private Boolean done = false;
  private int cone_number;
  
  private double target_y;
  private double target_distance;
  private double gyro_aci;

  private double calc_y;
  private double cross_distance = -1;
  private double needed_angle;
  public AlignForLeft(DriveSubsystem driveSubsystem, int cone_number ) {
    this.cone_number = cone_number;
    addRequirements(driveSubsystem);
  }


  @Override
  public void initialize() {
  // System.out.println("AUTO Align LEFT Start");    
  target = FlareVisionSubsystem.getBestTarget();
  gyro_aci = DriveSubsystem.m_gyro.getYaw();
  target_distance = FlareVisionSubsystem.getDistanceToGoal(target);
  target_y = FlareVisionSubsystem.getYdistance(target);
  }


  @Override
  public void execute() {
    
    if(FlareVisionSubsystem.getAprilTagID() == 8 || FlareVisionSubsystem.getAprilTagID() == 3){
      if(cross_distance <=0){
      calc_y = AlignConstants.cone_1_distance + target_y;
      cross_distance = Math.sqrt(Math.pow(calc_y,2)+Math.pow(target_distance,2))-10;
      needed_angle = Math.acos(target_distance/cross_distance);
    }else if(!DriveSubsystem.turn_angles(needed_angle, gyro_aci)){
     
    }
      
    

      



      
    }

  }


  @Override
  public void end(boolean interrupted) {
  
    // System.out.println("AUTO Align LEFT End");
  }

  @Override
  public boolean isFinished() {
    return false;
  }

  

}
