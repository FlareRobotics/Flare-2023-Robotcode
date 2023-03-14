package frc.robot.Custom;


import frc.robot.RobotContainer;
import frc.robot.subsystems.LedSubsystem;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class SupplyGather extends CommandBase {
    

    public SupplyGather(LedSubsystem subsystem) {
       

        addRequirements(subsystem);
    }

    @Override
    public void initialize() {
        //System.out.println("Supply Gather START");
        RobotContainer.wantedCone = !RobotContainer.wantedCone;
        if(RobotContainer.wantedCone){
        RobotContainer.currentState = RobotState.CubeWanted;
       }else{
        RobotContainer.currentState = RobotState.ConeWanted;
       }
    }

    @Override
    public void execute() {

    }

    @Override
    public void end(boolean interrupted) {
    //    System.out.println("SupplyGather End!");
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
