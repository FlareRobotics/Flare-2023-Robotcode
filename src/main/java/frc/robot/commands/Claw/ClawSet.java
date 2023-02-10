package frc.robot.commands.Claw;

import frc.robot.subsystems.ClawSubsystem;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class ClawSet extends CommandBase {
  private boolean claw_position;

  public ClawSet(ClawSubsystem subsystem, Boolean claw_position) {
    this.claw_position = claw_position;
    addRequirements(subsystem);
  }

  @Override
  public void initialize() {
    System.out.println("Claw Set Start!");
  }

  @Override
  public void execute() {
    if (claw_position) {
      ClawSubsystem.claw_open();
    } else {
      ClawSubsystem.claw_close();
    }
  }

  @Override
  public void end(boolean interrupted) {
    System.out.println("ClawSet End!");
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
