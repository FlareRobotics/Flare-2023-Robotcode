package frc.robot.Custom;

import frc.robot.RobotContainer;
import frc.robot.subsystems.LedSubsystem;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class RobotStateChanger extends CommandBase {
    public int istenen_state;

    public RobotStateChanger(LedSubsystem subsystem, int Robotstate) {
        istenen_state = Robotstate;
        addRequirements(subsystem);
    }

    @Override
    public void initialize() {
        System.out.println("Robot State Changer START");
        
        switch(istenen_state){
            case 0:
                RobotContainer.currentState = RobotState.LedCLose;
                break;
            case 1:
                RobotContainer.currentState = RobotState.Balanced;
                break;
            case 2:
                RobotContainer.currentState = RobotState.NotBalanced;
                break;
            }
    }

    @Override
    public void execute() {

    }

    @Override
    public void end(boolean interrupted) {
    //    System.out.println("Robot State changer End!");
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
