package frc.robot.Custom;

import frc.robot.RobotContainer;
import frc.robot.subsystems.LedSubsystem;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class SupplyGather extends CommandBase {
    private static int state = 0;

    public SupplyGather(LedSubsystem subsystem) {
        state++;
        if (state >= 3)
            state = 0;

        addRequirements(subsystem);
    }

    @Override
    public void initialize() {
        System.out.println("Claw Set Start!");

        switch (state) {
            case 0:
                RobotContainer.currentState = RobotState.None;
                break;

            case 1:
                RobotContainer.currentState = RobotState.ConeWanted;
                break;

            case 2:
                RobotContainer.currentState = RobotState.CubeWanted;
                break;

            default:
                break;
        }
    }

    @Override
    public void execute() {

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
