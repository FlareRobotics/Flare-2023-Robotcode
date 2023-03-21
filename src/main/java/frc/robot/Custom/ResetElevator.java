package frc.robot.Custom;

import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ClawSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class ResetElevator extends CommandBase {
    private boolean isReseted = false;

    public ResetElevator(ArmSubsystem armSubsystem, ElevatorSubsystem elevatorSubsystem, ClawSubsystem clawSubsystem) {
        addRequirements(armSubsystem, elevatorSubsystem, clawSubsystem);
    }

    @Override
    public void initialize() {
        System.out.println("Robot Reset Start!");
    }

    @Override
    public void execute() {
        ElevatorSubsystem.elevator_motor.setSelectedSensorPosition(0);
    }

    @Override
    public void end(boolean interrupted) {
        System.out.println("Robot Reset End!");
    }

    @Override
    public boolean isFinished() {
        return isReseted;
    }
}
