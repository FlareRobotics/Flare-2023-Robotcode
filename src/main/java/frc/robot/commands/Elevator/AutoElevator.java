package frc.robot.commands.Elevator;

import frc.robot.Constants.ElevatorConstants;
import frc.robot.subsystems.ElevatorSubsystem;
import com.ctre.phoenix.motorcontrol.ControlMode;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class AutoElevator extends CommandBase {
  public int yukseklik;
  private boolean finished = false;
  private double offsetLimit = 0.25d;

  public AutoElevator(ElevatorSubsystem elevatorSubsystem, int yukseklik) {
    this.yukseklik = yukseklik;
    addRequirements(elevatorSubsystem);
  }

  @Override
  public void initialize() {
    // System.out.println("AUTO Elevator Start");
  }

  @Override
  public void execute() {
    double goal = 0;
    switch (yukseklik) {
      // Alt
      case 1:
        goal = ElevatorConstants.bottom_row_height;
        break;
      // Orta
      case 2:
        goal = ElevatorConstants.middle_row_height;
        break;
      // Üst
      case 3:
        goal = ElevatorConstants.top_row_height;
        break;
      // Üst
      case 4:
        goal = ElevatorConstants.substation_height;
      break;
    }



    goal = goal- ElevatorSubsystem.elevator_motor.getSelectedSensorPosition()  ;
    System.out.println(goal);
    ElevatorSubsystem.elevator_motor.set(ControlMode.MotionMagic, goal);
  }

  @Override
  public void end(boolean interrupted) {
    ElevatorSubsystem.elevator_motor_set(0);
    // System.out.println("AUTO Elevator End");
  }

  @Override
  public boolean isFinished() {
    return finished;
  }
}