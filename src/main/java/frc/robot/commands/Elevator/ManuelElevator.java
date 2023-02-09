package frc.robot.commands.Elevator;

import frc.robot.Constants.ElevatorConstants;
import frc.robot.subsystems.ElevatorSubsystem;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class ManuelElevator extends CommandBase {
  public boolean yon;

  public ManuelElevator(ElevatorSubsystem elevatorSubsystem, boolean yon) {
    this.yon = yon;
    addRequirements(elevatorSubsystem);
  }

  @Override
  public void initialize() {
    // System.out.println("Manuel Elevator Start");
  }

  @Override
  public void execute() {
    if (yon) {
      ElevatorSubsystem.elevator_motor_set(ElevatorConstants.elevator_hiz * 1);
    } else {
      ElevatorSubsystem.elevator_motor_set(ElevatorConstants.elevator_hiz * -1);
    }
  }

  @Override
  public void end(boolean interrupted) {
    ElevatorSubsystem.elevator_motor_set(0);
    // System.out.println("Manuel Elevator End");
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
