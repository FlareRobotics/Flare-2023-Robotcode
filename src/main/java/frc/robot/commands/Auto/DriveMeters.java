package frc.robot.commands.Auto;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.Custom.RobotState;
import frc.robot.subsystems.DriveSubsystem;

public class DriveMeters extends CommandBase {

  /** Creates a new JoystickDriveCommand. */
  private final DriveSubsystem m_drive;

  private final double cm;

  public DriveMeters(
      DriveSubsystem drive,
      double meters) {
    m_drive = drive;
    cm = meters;
    addRequirements(m_drive);
  }

  @Override
  public void initialize() {
  }

  @Override
  public void execute() {
    DriveSubsystem.arcadeDrive(cm > 0 ? 0.75d : -0.75d, 0);
  }

  @Override
  public void end(boolean interrupted) {
    DriveSubsystem.arcadeDrive(0, 0);
    RobotContainer.currentState = RobotState.Balanced;
  }

  @Override
  public boolean isFinished() {
    return Math.abs(DriveSubsystem.getAverageEncoderDistance()) > Math.abs(cm - 6.25d);
  }
}