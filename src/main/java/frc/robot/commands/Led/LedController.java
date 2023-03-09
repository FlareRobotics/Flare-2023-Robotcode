package frc.robot.commands.Led;

import frc.robot.Custom.RobotState;
import frc.robot.subsystems.LedSubsystem;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class LedController extends CommandBase {
  private RobotState currentState = RobotState.ConePicked;
  private LedSubsystem ledSubsystem;

  public LedController(LedSubsystem subsystem, RobotState robotState) {
    
    this.currentState = robotState;
    this.ledSubsystem = subsystem;
    addRequirements(subsystem);
  }

  @Override
  public void initialize() {
    System.out.println("Led Controller Start!");
  }

  @Override
  public void execute() {
   System.out.println(currentState);
   /*switch (currentState) {
      case None:
        ledSubsystem.rainbow();
        break;

      case CubePicked:
        ledSubsystem.purplePulse();
        break;

      case ConePicked:
        ledSubsystem.yellowPulse();
        break;

      case CubeWanted:
        ledSubsystem.purple();
        break;

      case ConeWanted:
        ledSubsystem.yellow();
        break;

      case NotBalanced:
        ledSubsystem.red();
        break;

      case Balanced:
        ledSubsystem.green();
        break;

      default:
        break;
    }*/
    if(currentState == RobotState.ConeWanted){
      ledSubsystem.rainbow();
      System.out.println("aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa");
    }else if(currentState == RobotState.None){
      ledSubsystem.yellow();
      System.out.println("bbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbb");
    }
  }

  @Override
  public void end(boolean interrupted) {
    System.out.println("Led Controller End!");
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
