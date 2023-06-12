package frc.robot.commands.Elevator;


import frc.robot.Constants.ElevatorConstants;
import frc.robot.Custom.Distance_State;
import frc.robot.PID.PidConstants;
import frc.robot.subsystems.ElevatorSubsystem;

import com.ctre.phoenix.motorcontrol.TalonFXControlMode;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class AutoElevator extends CommandBase {
  public Distance_State yukseklik;
  private boolean finished = false;
  private double offsetLimit = 500;
  private double goal = 0;

  public AutoElevator(ElevatorSubsystem elevatorSubsystem, Distance_State yukseklik) {
    this.yukseklik = yukseklik;
    addRequirements(elevatorSubsystem);
  }

  @Override
  public void initialize() {
    System.out.println("AUTO Elevator Start");
    switch (yukseklik) {
      case Zero_All:
      goal = 0;
      break;
      // Alt
      case Middle_Cube_Elevator:
        goal = ElevatorConstants.middle_row_height_cube;
        break;
      // Orta
      case Middle_Cone_Elevator:
        goal = ElevatorConstants.middle_row_height;
        break;
      case Middle_Cone_Elevator_Auto:
        goal = ElevatorConstants.middle_row_height;
        break;
      case Substation_Elevator:
        goal = ElevatorConstants.substation_height;
        break;
      case Pick_Cube_Elevator:
        goal = -26000;
        break;
      case Cube_Above_Elevator:
        goal = 120000;
        break;
      default:
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
      ElevatorSubsystem.elevator_motor.configMotionCruiseVelocity(10000, PidConstants.TurretConstants.kTimeoutMs);
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