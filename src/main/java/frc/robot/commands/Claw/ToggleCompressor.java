// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Claw;

import frc.robot.subsystems.ClawSubsystem;
import edu.wpi.first.wpilibj2.command.CommandBase;

/** An example command that uses an example subsystem. */
public class ToggleCompressor extends CommandBase {

  public ToggleCompressor(ClawSubsystem subsystem) {
    
    addRequirements(subsystem);
  }

  @Override
  public void initialize() {
    System.out.println("Toggle Compressor Start!");
  }


  @Override
  public void execute() {
    ClawSubsystem.Compressor.enableDigital();
  }


  @Override
  public void end(boolean interrupted) {
      ClawSubsystem.Compressor.disable();
    System.out.println("Toggle Compressor End!");
  }


  @Override
  public boolean isFinished() {
    return false;
  }
}
