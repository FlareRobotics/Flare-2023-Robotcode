package frc.robot.commands.Auto;

import frc.robot.RobotContainer;
import frc.robot.Constants.AutoConstants;
import frc.robot.Custom.RobotState;
import frc.robot.subsystems.DriveSubsystem;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class AutobalanceCommand extends CommandBase {

  public AutobalanceCommand(DriveSubsystem subsystem) {
    addRequirements(subsystem);
  }

  @Override
  public void initialize() {
    System.out.println("AutoBalance Start!");
  }

  @Override
  public void execute() {
    if (DriveSubsystem.m_gyro.getPitch() > AutoConstants.pitch_tolerance) {
      RobotContainer.currentState = RobotState.NotBalanced;
      DriveSubsystem.arcadeDrive(-0.3, 0);
    } else if (DriveSubsystem.m_gyro.getPitch() < -AutoConstants.pitch_tolerance) {
      RobotContainer.currentState = RobotState.NotBalanced;
      DriveSubsystem.arcadeDrive(0.3, 0);
    } else {
      RobotContainer.currentState = RobotState.Balanced;
      DriveSubsystem.arcadeDrive(0, 0);
    }
  }

  @Override
  public void end(boolean interrupted) {
    System.out.println("AutoBalance END!");
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
