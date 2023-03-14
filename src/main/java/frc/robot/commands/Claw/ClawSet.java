package frc.robot.commands.Claw;

import frc.robot.RobotContainer;
import frc.robot.Custom.RobotState;
import frc.robot.subsystems.ClawSubsystem;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class ClawSet extends CommandBase {
  private boolean cone;

  public ClawSet(ClawSubsystem subsystem, boolean cone) {
    this.cone = cone;
    RobotContainer.clawOpen = !RobotContainer.clawOpen;

    addRequirements(subsystem);
  }

  @Override
  public void initialize() {
    System.out.println("Claw Set Start!");
    if (cone){
  RobotContainer.currentState = RobotState.ConePicked;
    }else{
      RobotContainer.currentState = RobotState.CubePicked;
    }
  }

  @Override
  public void execute() {
    if(cone){
      ClawSubsystem.claw_close_cone();
    }else{
      ClawSubsystem.claw_close_cube();
    }

      
    
  }

  @Override
  public void end(boolean interrupted) {
    System.out.println("ClawSet End!");
    ClawSubsystem.claw_open();  
    RobotContainer.currentState = RobotState.None;
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
