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
    switch (yukseklik) {
      // Alt
      case 1:
        ElevatorSubsystem.elevator_motor.set(ControlMode.MotionMagic,
            ElevatorSubsystem.elevator_yukseklik_units(ElevatorConstants.bottom_row_height));
            finished = ElevatorSubsystem.elevator_yukseklik_cm() >= ElevatorConstants.bottom_row_height - offsetLimit && ElevatorSubsystem.elevator_yukseklik_cm() <= ElevatorConstants.bottom_row_height + offsetLimit;
        break;
      // Orta
      case 2:
        ElevatorSubsystem.elevator_motor.set(ControlMode.MotionMagic,
            ElevatorSubsystem.elevator_yukseklik_units(ElevatorConstants.middle_row_height));
            finished = ElevatorSubsystem.elevator_yukseklik_cm() >= ElevatorConstants.middle_row_height - offsetLimit && ElevatorSubsystem.elevator_yukseklik_cm() <= ElevatorConstants.middle_row_height + offsetLimit;
        break;
      // Üst
      case 3:
        ElevatorSubsystem.elevator_motor.set(ControlMode.MotionMagic,
            ElevatorSubsystem.elevator_yukseklik_units(ElevatorConstants.top_row_height));
            finished = ElevatorSubsystem.elevator_yukseklik_cm() >= ElevatorConstants.top_row_height - offsetLimit && ElevatorSubsystem.elevator_yukseklik_cm() <= ElevatorConstants.top_row_height + offsetLimit;
        break;
      // Üst
      case 4:
      ElevatorSubsystem.elevator_motor.set(ControlMode.MotionMagic,
          ElevatorSubsystem.elevator_yukseklik_units(ElevatorConstants.substation_height));
          finished = ElevatorSubsystem.elevator_yukseklik_cm() >= ElevatorConstants.substation_height - offsetLimit && ElevatorSubsystem.elevator_yukseklik_cm() <= ElevatorConstants.substation_height + offsetLimit;
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
    return finished;
  }
}