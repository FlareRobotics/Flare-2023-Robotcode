// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Arm;

import frc.robot.Constants.ArmConstants;
import frc.robot.subsystems.ArmSubsystem;
import edu.wpi.first.wpilibj2.command.CommandBase;

/** An example command that uses an example subsystem. */
public class ManuelArm extends CommandBase {

  private Boolean yon;

  public ManuelArm(ArmSubsystem subsystem, Boolean yon) {
    this.yon = yon;
    addRequirements(subsystem);
  }

  @Override
  public void initialize() {
    // System.out.println("Manuel Arm Start");
  }

  @Override
  public void execute() {
    if (yon) {
      ArmSubsystem.arm_motor.set(ArmConstants.arm_hiz);
    } else {
      ArmSubsystem.arm_motor.set(ArmConstants.arm_hiz * -1);
    }
  }

  @Override
  public void end(boolean interrupted) {
    // System.out.println("Manuel Arm End");
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
