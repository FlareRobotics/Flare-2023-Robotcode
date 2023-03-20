package frc.robot.commands.Arm;

import frc.robot.Constants;
import frc.robot.Constants.ArmConstants;
import frc.robot.subsystems.ArmSubsystem;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class AutoArm extends CommandBase {
  public int yukseklik;
  private boolean finished = false;
  private double offsetLimit = 1000;

  public AutoArm(ArmSubsystem armSubsystem, int yukseklik) {
    this.yukseklik = yukseklik;
    addRequirements(armSubsystem);
  }

  @Override
  public void initialize() {
    System.out.println("AUTO Arm Start");
  }

  @Override
  public void execute() {
    double goal = 0;
    switch (yukseklik) {
      // Alt
      case 1:
        goal = ArmConstants.bottom_row_distance;
        break;
      // Orta
      case 2:
        goal = ArmConstants.middle_row_distance;
        break;
      // Üst
      case 3:
        goal = ArmConstants.top_row_distance;
        break;
      // Üst
      case 4:
        goal = ArmConstants.substation_distance;
      break;
    }


    goal = goal - ArmSubsystem.arm_motor.getSelectedSensorPosition();
    ArmSubsystem.arm_motor.set(Constants.ArmConstants.arm_speed * goal < 0 ? -1 : 1);

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