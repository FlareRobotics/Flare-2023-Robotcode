package frc.robot.commands.Elevator;


import frc.robot.Constants.ElevatorConstants;
import frc.robot.subsystems.ElevatorSubsystem;

import com.ctre.phoenix.motorcontrol.TalonFXControlMode;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class AutoElevator extends CommandBase {
  public int yukseklik;
  private boolean finished = false;
  private double offsetLimit = 1000;

  public AutoElevator(ElevatorSubsystem elevatorSubsystem, int yukseklik) {
    this.yukseklik = yukseklik;
    addRequirements(elevatorSubsystem);
  }

  @Override
  public void initialize() {
    System.out.println("AUTO Elevator Start");
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


    goal = goal - ElevatorSubsystem.elevator_motor.getSelectedSensorPosition();
    ElevatorSubsystem.elevator_motor.set(TalonFXControlMode.MotionMagic, goal);
    finished = Math.abs(ElevatorSubsystem.elevator_motor.getSelectedSensorPosition() - goal) <= offsetLimit;
  }

  @Override
  public void end(boolean interrupted) {
    System.out.println("Auto elevator end");
    ElevatorSubsystem.elevator_motor_set(0);
  }

  @Override
  public boolean isFinished() {
    return finished;
  }
}