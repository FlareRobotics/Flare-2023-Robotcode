package frc.robot.commands.Arm;


import frc.robot.Constants.ArmConstants;
import frc.robot.PID.PidConstants;
import frc.robot.subsystems.ArmSubsystem;

import com.ctre.phoenix.motorcontrol.TalonFXControlMode;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class AutoArm extends CommandBase {
  public int yukseklik;
  private boolean finished = false;
  private double offsetLimit = 400;
  private double goal = 0;

  public AutoArm(ArmSubsystem armSubsystem, int yukseklik) {
    this.yukseklik = yukseklik;
    addRequirements(armSubsystem);
  }

  @Override
  public void initialize() {
    System.out.println("AUTO arm Start");
    switch (yukseklik) {
      case -1:
      goal = 0;
      break;
      // Alt
      case 1:
        goal = ArmConstants.middle_row_distance;
        break;
      case 2:
        goal = ArmConstants.middle_row_distance_Cone;
        break;
      case 3:
        goal = 160000;
        break;
    }
  }

  @Override
  public void execute() {
    if(goal > ArmSubsystem.arm_motor.getSelectedSensorPosition())
    {
      ArmSubsystem.arm_motor.configMotionCruiseVelocity(15000, PidConstants.TurretConstants.kTimeoutMs);
      ArmSubsystem.arm_motor.configMotionAcceleration(18000, PidConstants.TurretConstants.kTimeoutMs);
    }
    else
    {
      ArmSubsystem.arm_motor.configMotionCruiseVelocity(15000, PidConstants.TurretConstants.kTimeoutMs);
      ArmSubsystem.arm_motor.configMotionAcceleration(18000, PidConstants.TurretConstants.kTimeoutMs);
    }
    ArmSubsystem.arm_motor.set(TalonFXControlMode.MotionMagic, goal);
    finished = Math.abs(ArmSubsystem.arm_motor.getSelectedSensorPosition() - goal) <= offsetLimit;
  }

  @Override
  public void end(boolean interrupted) {
    System.out.println("Auto arm end");
    ArmSubsystem.arm_motor_set(0);
  }

  @Override
  public boolean isFinished() {
    return finished;
  }
}