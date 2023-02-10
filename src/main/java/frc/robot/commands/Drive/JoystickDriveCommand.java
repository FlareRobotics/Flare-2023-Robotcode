package frc.robot.commands.Drive;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;
import java.util.function.DoubleSupplier;

public class JoystickDriveCommand extends CommandBase {

  /** Creates a new JoystickDriveCommand. */
  private final DriveSubsystem m_drive;

  private final DoubleSupplier m_left;
  private final DoubleSupplier m_right;

  public JoystickDriveCommand(
      DriveSubsystem drive,
      DoubleSupplier left,
      DoubleSupplier right) {
    m_drive = drive;
    m_left = left;
    m_right = right;
    addRequirements(m_drive);
  }

  @Override
  public void initialize() {
  }

  @Override
  public void execute() {
    DriveSubsystem.arcadeDrive(m_left.getAsDouble(), m_right.getAsDouble());
  }

  @Override
  public void end(boolean interrupted) {
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}