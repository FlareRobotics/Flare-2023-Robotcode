package frc.robot.commands.Claw;

import frc.robot.RobotContainer;
import frc.robot.Custom.RobotState;
import frc.robot.subsystems.ClawSubsystem;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class ClawSet extends CommandBase {
  private boolean cone;

  public ClawSet(ClawSubsystem subsystem, boolean cone) {
    this.cone = cone;
    addRequirements(subsystem);
  }

  @Override
  public void initialize() {
    System.out.println("Claw Set Start!");
    RobotContainer.clawOpen = !RobotContainer.clawOpen;
    RobotContainer.currentState = RobotState.None;
  }

  @Override
  public void execute() {
      ClawSubsystem.claw_close();
  
  }
  @Override
  public void end(boolean interrupted) {
    System.out.println("ClawSet End!");
    ClawSubsystem.claw_open();
    if (cone) {
      RobotContainer.currentState = RobotState.ConePicked;
    } else {
      RobotContainer.currentState = RobotState.CubePicked;
    }
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
