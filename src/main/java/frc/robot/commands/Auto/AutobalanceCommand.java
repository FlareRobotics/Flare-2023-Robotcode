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
    if (DriveSubsystem.m_gyro.getPitch() > 0 + AutoConstants.pitch_tolerance) {
      RobotContainer.currentState = RobotState.NotBalanced;
      DriveSubsystem.pidDrive(0.2, 0);
    } else if (DriveSubsystem.m_gyro.getPitch() < (0 - AutoConstants.pitch_tolerance)) {
      RobotContainer.currentState = RobotState.NotBalanced;
      DriveSubsystem.pidDrive(-0.2, 0);
    } else {
      RobotContainer.currentState = RobotState.Balanced;
      DriveSubsystem.pidDrive(0, 0);
    }
  }

  @Override
  public void end(boolean interrupted) {
    System.out.println("AutoBalance END!");
  RobotContainer.currentState = RobotState.None;
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
