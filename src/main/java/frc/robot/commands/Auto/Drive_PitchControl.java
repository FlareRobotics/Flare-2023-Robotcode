package frc.robot.commands.Auto;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;

public class Drive_PitchControl extends CommandBase {
  private final DriveSubsystem m_drive;

  private boolean yon;

  public Drive_PitchControl(DriveSubsystem drive, boolean yon){
    m_drive = drive;
    this.yon = yon;
    addRequirements(m_drive);
  }

  @Override
  public void initialize() {
  }

  @Override
  public void execute() {
    DriveSubsystem.arcadeDrive(yon ? 0.5d : 0.5d,0);
  }

  @Override
  public void end(boolean interrupted) {
    DriveSubsystem.arcadeDrive(0, 0);
  }

  @Override
  public boolean isFinished() {
    return DriveSubsystem.m_gyro.getPitch() < -10;
  }
}