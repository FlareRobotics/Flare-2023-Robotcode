package frc.robot.commands.Arm;


import frc.robot.Constants.ArmConstants;
import frc.robot.subsystems.ArmSubsystem;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class AutoArm extends CommandBase {
  public int yukseklik;
  private double goal = 0;

  public AutoArm(ArmSubsystem armSubsystem, int yukseklik) {
    this.yukseklik = yukseklik;
    addRequirements(armSubsystem);
  }

  @Override
  public void initialize() {
    System.out.println("AUTO Arm Start");
    switch (yukseklik) {
      case -1:
      goal = 0;
      break;
      // Alt
      case 1:
        goal = ArmConstants.middle_row_distance;
        break;
    }
  }

  @Override
  public void execute() {
    ArmSubsystem.arm_motor.set(ArmConstants.arm_speed);
  }

  @Override
  public void end(boolean interrupted) {
    System.out.println("Auto Arm end");
    ArmSubsystem.arm_motor.set(0);
  }

  @Override
  public boolean isFinished() {
    return ArmSubsystem.arm_motor.getSelectedSensorPosition() > goal;
  }
}