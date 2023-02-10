package frc.robot.commands.Arm;

import frc.robot.Constants.ArmConstants;
import frc.robot.subsystems.ArmSubsystem;
import com.ctre.phoenix.motorcontrol.ControlMode;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class AutoArm extends CommandBase {
  public int yukseklik;

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
        break;
      // Orta
      case 2:
        ArmSubsystem.arm_motor.set(ControlMode.MotionMagic,
            ArmSubsystem.arm_uzunluk_units(ArmConstants.middle_row_distance));
        break;
      // Üst
      case 3:
        ArmSubsystem.arm_motor.set(ControlMode.MotionMagic,
            ArmSubsystem.arm_uzunluk_units(ArmConstants.top_row_distance));
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
    return false;
  }
}
