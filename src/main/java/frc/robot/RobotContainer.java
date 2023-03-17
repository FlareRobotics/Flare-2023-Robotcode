package frc.robot;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Custom.RobotState;
import frc.robot.commands.Arm.ManuelArm;
import frc.robot.commands.Auto.FlareTrajectory;
import frc.robot.commands.Claw.ClawSet;
import frc.robot.commands.Claw.ToggleCompressor;
import frc.robot.commands.Drive.JoystickDriveCommand;
import frc.robot.commands.Elevator.ManuelElevator;
import frc.robot.commands.Led.LedController;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ClawSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.LedSubsystem;

//8054 <3
public class RobotContainer {

  public static XboxController driver_main = new XboxController(0);
  public static XboxController driver_2 = new XboxController(1);
  private static final ElevatorSubsystem elevatorsubsystem = new ElevatorSubsystem();
  private static final ClawSubsystem clawSubsystem = new ClawSubsystem();
  private static final ArmSubsystem armSubsystem = new ArmSubsystem();
  private static final DriveSubsystem m_robotDrive = new DriveSubsystem();
  public final static LedSubsystem ledSubsystem = new LedSubsystem();

  public static SendableChooser<Integer> autoChooser = new SendableChooser<>();

  public static boolean clawOpen = false;
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
            () -> -driver_main.getLeftY(),
            () -> driver_main.getRightX())));
  }

  private void configureButtonBindings() {
    // Manuel Elevator
    new JoystickButton(driver_main, XboxController.Button.kA.value)
        .whileTrue(new ManuelElevator(elevatorsubsystem, true));
    new JoystickButton(driver_main, XboxController.Button.kY.value)
        .whileTrue(new ManuelElevator(elevatorsubsystem, false));

    // Manuel Arm
    new JoystickButton(driver_main, XboxController.Button.kX.value).whileTrue(new ManuelArm(armSubsystem, true));
    new JoystickButton(driver_main, XboxController.Button.kB.value).whileTrue(new ManuelArm(armSubsystem, false));

    // Compressor Toggle
    new JoystickButton(driver_main, XboxController.Button.kStart.value)
        .toggleOnTrue(new ToggleCompressor(clawSubsystem));

    // Claw For Cone
    new JoystickButton(driver_main, XboxController.Button.kRightBumper.value)
        .toggleOnTrue(new ClawSet(clawSubsystem, true));
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