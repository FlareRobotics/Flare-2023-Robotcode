package frc.robot.commands.Auto;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;

public class DriveMetersPID extends CommandBase {

  private final double cm;

  public DriveMetersPID(DriveSubsystem m_drive, double meters) {
    cm = meters;
    addRequirements(m_drive);
  }

  @Override
  public void initialize() {
  }

  @Override
  public void execute() {
    DriveSubsystem.drive_PID_centimeters(cm);
  }

  @Override
  public void end(boolean interrupted) {
    DriveSubsystem.arcadeDrive(0, 0);
  }

  @Override
  public boolean isFinished() {
    return Math.abs(DriveSubsystem.getAverageEncoderDistance()) > Math.abs(cm);
  }
}