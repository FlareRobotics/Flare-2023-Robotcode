package frc.robot.Custom;

import frc.robot.RobotContainer;
import frc.robot.subsystems.LedSubsystem;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class SupplyGather extends CommandBase {
    private static boolean cone = false;

    public SupplyGather(LedSubsystem subsystem) {
        cone = !cone;

        addRequirements(subsystem);
    }

    @Override
    public void initialize() {
        System.out.println("Supply Gather START");
        RobotContainer.currentState = RobotState.ConePicked;
        //RobotContainer.currentState = cone ? RobotState.ConeWanted : RobotState.CubeWanted;
    }

    @Override
    public void execute() {

    }

    @Override
    public void end(boolean interrupted) {
        System.out.println("SupplyGather End!");
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}