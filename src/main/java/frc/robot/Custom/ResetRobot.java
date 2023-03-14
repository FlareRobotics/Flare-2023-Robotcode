package frc.robot.Custom;

import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ClawSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;

import com.ctre.phoenix.motorcontrol.ControlMode;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class ResetRobot extends CommandBase {
    private boolean isReseted = false;

    public ResetRobot(ArmSubsystem armSubsystem, ElevatorSubsystem elevatorSubsystem, ClawSubsystem clawSubsystem) {
        addRequirements(armSubsystem, elevatorSubsystem, clawSubsystem);
    }

    @Override
    public void initialize() {
        System.out.println("Robot Reset Start!");
    }

    @Override
    public void execute() {
        ArmSubsystem.arm_motor.set(ControlMode.MotionMagic,
                ArmSubsystem.arm_uzunluk_units(0));

        ElevatorSubsystem.elevator_motor.set(ControlMode.MotionMagic,
                ElevatorSubsystem.elevator_yukseklik_units(0));

        ClawSubsystem.claw_open();

        isReseted = ArmSubsystem.arm_uzunluk_cm() <= 1d && ElevatorSubsystem.elevator_yukseklik_cm() <= 1d;
    }

    @Override
    public void end(boolean interrupted) {
        System.out.println("Robot Reset End!");
    }

    @Override
    public boolean isFinished() {
        return isReseted;
    }
}
