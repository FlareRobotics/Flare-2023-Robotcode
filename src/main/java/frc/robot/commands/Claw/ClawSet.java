package frc.robot.commands.Claw;

import frc.robot.RobotContainer;
import frc.robot.Custom.RobotState;
import frc.robot.subsystems.ClawSubsystem;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class ClawSet extends CommandBase {
  public static RobotState lastState = RobotState.None;

  public ClawSet(ClawSubsystem subsystem) {
    addRequirements(subsystem);
  }

  @Override
  public void initialize() {
    System.out.println("Claw Set Start!");

    RobotContainer.clawOpen = !RobotContainer.clawOpen;

    if (RobotContainer.currentState == RobotState.ConeWanted || RobotContainer.currentState == RobotState.CubeWanted) {
      lastState = RobotContainer.currentState;
    }
    RobotContainer.currentState = lastState;
  }

  @Override
  public void execute() {
    ClawSubsystem.claw_close();
  }

  @Override
  public void end(boolean interrupted) {
    System.out.println("ClawSet End!");
    ClawSubsystem.claw_open();
    if (lastState == RobotState.ConeWanted) {
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
