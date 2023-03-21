package frc.robot;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Custom.ResetRobot;
import frc.robot.Custom.RobotState;
import frc.robot.Custom.SupplyGather;
import frc.robot.commands.Arm.AutoArm;
import frc.robot.commands.Arm.ManuelArm;
import frc.robot.commands.Auto.FlareTrajectory;
import frc.robot.commands.Claw.ClawSet;
import frc.robot.commands.Claw.ToggleCompressor;
import frc.robot.commands.Drive.JoystickDriveCommand;
import frc.robot.commands.Elevator.AutoElevator;
import frc.robot.commands.Elevator.ManuelElevator;
import frc.robot.commands.Led.LedController;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ClawSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.LedSubsystem;

//8054 <3
public class RobotContainer {

  public static XboxController driver_main = new XboxController(1);
  public static XboxController driver_2 = new XboxController(2);
  private static final ElevatorSubsystem elevatorsubsystem = new ElevatorSubsystem();
  private static final ClawSubsystem clawSubsystem = new ClawSubsystem();
  private static final ArmSubsystem armSubsystem = new ArmSubsystem();
  private static final DriveSubsystem m_robotDrive = new DriveSubsystem();
  public final static LedSubsystem ledSubsystem = new LedSubsystem();

  public static SendableChooser<Integer> autoChooser = new SendableChooser<>();

  public static boolean clawOpen = true;
  public static RobotState currentState = RobotState.None;
  public static boolean wantedCone = false;

  public RobotContainer() {
    configureButtonBindings();
    autoChooser.setDefaultOption("No Auto", 0);
    autoChooser.addOption("1 Piece Forward + TAXI", 1);
    autoChooser.addOption("1 Piece Forward + Auto Balance", 2);
    autoChooser.addOption("Auto Balance", 3);
    SmartDashboard.putData(autoChooser);

    ledSubsystem.setDefaultCommand(new LedController(ledSubsystem));
    m_robotDrive.setDefaultCommand(
        new ParallelCommandGroup(new JoystickDriveCommand(
            m_robotDrive,
            () -> -driver_main.getLeftY()/1.5,
            () -> -driver_main.getRightX()/2)));
  }

  private void configureButtonBindings() {
    // Manuel Elevator
    new JoystickButton(driver_2, 8)
        .whileTrue(new ManuelElevator(elevatorsubsystem, true));
    new JoystickButton(driver_2, 7)
        .whileTrue(new ManuelElevator(elevatorsubsystem, false));

    // Manuel Arm
    new JoystickButton(driver_main, XboxController.Button.kX.value).whileTrue(new ManuelArm(armSubsystem, true));
    new JoystickButton(driver_main, XboxController.Button.kB.value).whileTrue(new ManuelArm(armSubsystem, false));

    // Compressor Toggle
    new JoystickButton(driver_main, XboxController.Button.kStart.value)
        .toggleOnTrue(new ToggleCompressor(clawSubsystem));

    // Claw For Cone
    new JoystickButton(driver_main, XboxController.Button.kRightBumper.value)
        .toggleOnTrue(new ClawSet(clawSubsystem, currentState == RobotState.ConeWanted));

    // Wanted Status
  
    new JoystickButton(driver_2, 5).toggleOnTrue(new SupplyGather(ledSubsystem));

    new JoystickButton(driver_2, 9).toggleOnTrue(new ResetRobot(armSubsystem, elevatorsubsystem, clawSubsystem));

    new JoystickButton(driver_2,3).whileTrue(new AutoElevator(elevatorsubsystem, -1));

    new JoystickButton(driver_2, 2).whileTrue(new AutoElevator(elevatorsubsystem, 1));
  
    new JoystickButton(driver_2, 1).whileTrue(new AutoElevator(elevatorsubsystem, 2));
  }

  public static Command getAuto() {
    switch (autoChooser.getSelected()) {
      case 0:
        return null;
      case 1:
        return null;
      case 2:
        // Flare Trajectory Example
        return new FlareTrajectory(m_robotDrive, elevatorsubsystem, armSubsystem, clawSubsystem);
      case 3:
        return null;
      default:
        return null;
    }
  }

  public Command getAutonomousCommand() {
    return null;
  }
}
