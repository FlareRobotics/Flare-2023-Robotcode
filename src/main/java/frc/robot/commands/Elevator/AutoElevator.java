package frc.robot.commands.Elevator;

import frc.robot.Constants.ElevatorConstants;
import frc.robot.subsystems.ElevatorSubsystem;
import com.ctre.phoenix.motorcontrol.ControlMode;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class AutoElevator extends CommandBase {
  public int yukseklik;

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
    switch (yukseklik) {
      // Alt
      case 1:
        ElevatorSubsystem.elevator_motor.set(ControlMode.MotionMagic,
            ElevatorSubsystem.elevator_yukseklik_units(ElevatorConstants.bottom_row_height));
        break;
      // Orta
      case 2:
        ElevatorSubsystem.elevator_motor.set(ControlMode.MotionMagic,
            ElevatorSubsystem.elevator_yukseklik_units(ElevatorConstants.middle_row_height));
        break;
      // Üst
      case 3:
        ElevatorSubsystem.elevator_motor.set(ControlMode.MotionMagic,
            ElevatorSubsystem.elevator_yukseklik_units(ElevatorConstants.top_row_height));
        break;
    }
  }

  @Override
  public void end(boolean interrupted) {
    ElevatorSubsystem.elevator_motor_set(0);
    // System.out.println("AUTO Elevator End");
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
