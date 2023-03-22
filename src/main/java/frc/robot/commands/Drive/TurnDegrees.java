package frc.robot.commands.Drive;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;

public class TurnDegrees extends CommandBase {

  /** Creates a new JoystickDriveCommand. */
  private final DriveSubsystem m_drive;

  private final double angle;

  public TurnDegrees(
      DriveSubsystem drive,
      double ang) {
    m_drive = drive;
    angle = ang;
    addRequirements(m_drive);
  }

  @Override
  public void initialize() {
  }

  @Override
  public void execute() {
    DriveSubsystem.turnDegrees(angle);
  }

  @Override
  public void end(boolean interrupted) {
    DriveSubsystem.arcadeDrive(0, 0);
  }

  @Override
  public boolean isFinished() {
    return Math.abs(DriveSubsystem.m_gyro.getYaw()) > angle - 3;
  }
}