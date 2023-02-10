package frc.robot.commands.Arm;

import frc.robot.Constants.ArmConstants;
import frc.robot.subsystems.ArmSubsystem;
import com.ctre.phoenix.motorcontrol.ControlMode;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class AutoArm extends CommandBase {
  public int yukseklik;
  private boolean finished = false;
  private double offsetLimit = 0.25d;

  public AutoArm(ArmSubsystem armSubsytem, int yukseklik) {
    this.yukseklik = yukseklik;
    addRequirements(armSubsytem);
  }

  @Override
  public void initialize() {
    // System.out.println("AUTO Arm Start");
  }

  @Override
  public void execute() {
    switch (yukseklik) {
      // Alt
      case 1:
        ArmSubsystem.arm_motor.set(ControlMode.MotionMagic,
            ArmSubsystem.arm_uzunluk_units(ArmConstants.bottom_row_distance));
            finished = ArmSubsystem.arm_uzunluk_cm() >= ArmConstants.bottom_row_distance - offsetLimit && ArmSubsystem.arm_uzunluk_cm() <= ArmConstants.bottom_row_distance + offsetLimit;
        break;
      // Orta
      case 2:
        ArmSubsystem.arm_motor.set(ControlMode.MotionMagic,
            ArmSubsystem.arm_uzunluk_units(ArmConstants.middle_row_distance));
            finished = ArmSubsystem.arm_uzunluk_cm() >= ArmConstants.middle_row_distance - offsetLimit && ArmSubsystem.arm_uzunluk_cm() <= ArmConstants.middle_row_distance + offsetLimit;
        break;
      // Üst
      case 3:
        ArmSubsystem.arm_motor.set(ControlMode.MotionMagic,
            ArmSubsystem.arm_uzunluk_units(ArmConstants.top_row_distance));
            finished = ArmSubsystem.arm_uzunluk_cm() >= ArmConstants.top_row_distance - offsetLimit && ArmSubsystem.arm_uzunluk_cm() <= ArmConstants.top_row_distance + offsetLimit;
        break;
        // Sub Station
      case 4:
      ArmSubsystem.arm_motor.set(ControlMode.MotionMagic,
          ArmSubsystem.arm_uzunluk_units(ArmConstants.substation_distance));
          finished = ArmSubsystem.arm_uzunluk_cm() >= ArmConstants.substation_distance - offsetLimit && ArmSubsystem.arm_uzunluk_cm() <= ArmConstants.substation_distance + offsetLimit;
      break;
    }
  }

  @Override
  public void end(boolean interrupted) {
    ArmSubsystem.arm_motor_set(0);
    // System.out.println("AUTO Arm End");
  }

  @Override
  public boolean isFinished() {
    return finished;
  }
}