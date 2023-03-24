package frc.robot.commands.Elevator;


import frc.robot.Constants.ElevatorConstants;
import frc.robot.PID.PidConstants;
import frc.robot.subsystems.ElevatorSubsystem;

import com.ctre.phoenix.motorcontrol.TalonFXControlMode;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class AutoElevator extends CommandBase {
  public int yukseklik;
  private boolean finished = false;
  private double offsetLimit = 500;
  private double goal = 0;

  public AutoElevator(ElevatorSubsystem elevatorSubsystem, int yukseklik) {
    this.yukseklik = yukseklik;
    addRequirements(elevatorSubsystem);
  }

  @Override
  public void initialize() {
    System.out.println("AUTO Elevator Start");
    switch (yukseklik) {
      case -1:
      goal = 0;
      break;
      // Alt
      case 1:
        goal = ElevatorConstants.middle_row_height_cube;
        break;
      // Orta
      case 2:
        goal = ElevatorConstants.middle_row_height;
        break;
      case 3:
        goal = ElevatorConstants.middle_row_height - 110000;
        break;
      case 4:
        goal = -26000;
        break;
      case 5:
        goal = 45000;
        break;
    }
  }

  @Override
  public void execute() {
    if(goal > ElevatorSubsystem.elevator_motor.getSelectedSensorPosition())
    {
      ElevatorSubsystem.elevator_motor.configMotionCruiseVelocity(15000, PidConstants.TurretConstants.kTimeoutMs);
      ElevatorSubsystem.elevator_motor.configMotionAcceleration(18000, PidConstants.TurretConstants.kTimeoutMs);
    }
    else
    {
      ElevatorSubsystem.elevator_motor.configMotionCruiseVelocity(7500, PidConstants.TurretConstants.kTimeoutMs);
      ElevatorSubsystem.elevator_motor.configMotionAcceleration(15000, PidConstants.TurretConstants.kTimeoutMs);
    }
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