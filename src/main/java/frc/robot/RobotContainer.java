package frc.robot;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Custom.ResetRobot;
import frc.robot.Custom.RobotState;
import frc.robot.Custom.RobotStateChanger;
import frc.robot.Custom.SupplyGather;
import frc.robot.commands.Arm.AutoArm;
import frc.robot.commands.Arm.ManuelArm;
import frc.robot.commands.Auto.AutobalanceCommand;
import frc.robot.commands.Auto.DriveMeters;
import frc.robot.commands.Auto.Drive_PitchControl;
import frc.robot.commands.Claw.ClawSet;
import frc.robot.commands.Claw.ToggleCompressor;
import frc.robot.commands.Drive.JoystickDriveCommand;
import frc.robot.commands.Drive.TurnDegrees;
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
    autoChooser.addOption("Mobility", 1);
    autoChooser.addOption("Mobility + Balance", 2);
    autoChooser.addOption("Balance", 3);
    autoChooser.addOption("Middle Cube + Mobility", 4);
    autoChooser.addOption("Middle Cube + Balance", 5);
    autoChooser.addOption("Middle Cube", 6);
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
        .toggleOnTrue(new ClawSet(clawSubsystem));

    new JoystickButton(driver_main, XboxController.Button.kLeftBumper.value)
        .whileTrue(new TurnDegrees(m_robotDrive, 90));

    // Wanted Status
    new JoystickButton(driver_2, 5).toggleOnTrue(new SupplyGather(ledSubsystem));
    // Reset Encoders
    new JoystickButton(driver_2, 9).whileTrue(new ResetRobot(armSubsystem, elevatorsubsystem, clawSubsystem));
    // Home Elevator PID
    new JoystickButton(driver_2,3).whileTrue(new AutoElevator(elevatorsubsystem, -1));

    new JoystickButton(driver_2,4).whileTrue(new DriveMeters(m_robotDrive, 100));
    //Auto Elevator
    new JoystickButton(driver_2, 2).whileTrue(new AutoElevator(elevatorsubsystem, 1));

    new JoystickButton(driver_2, 1).whileTrue(new AutoElevator(elevatorsubsystem, 2));
    
    // Led  Close
    new JoystickButton(driver_2, 11).whileTrue(new RobotStateChanger(ledSubsystem,0));
  }

  public static Command getAuto() {
    switch (autoChooser.getSelected()) {
      case 0:
        return new RobotStateChanger(ledSubsystem, 2);
      case 1:
        return 
        new SequentialCommandGroup(new DriveMeters(m_robotDrive, -400), 
        new RobotStateChanger(ledSubsystem, 1) );
      case 2:
        return 
        new SequentialCommandGroup(new DriveMeters(m_robotDrive, -400), 
        new Drive_PitchControl(m_robotDrive, true),
        new AutobalanceCommand(m_robotDrive));
      case 3:
        return 
        new SequentialCommandGroup(new Drive_PitchControl(m_robotDrive, false),
        new AutobalanceCommand(m_robotDrive));

      case 4:
      return
      new SequentialCommandGroup(new AutoElevator(elevatorsubsystem, 2),
      new AutoArm(armSubsystem, 1),
      new ClawSet(clawSubsystem).withTimeout(0.5),
      new AutoArm(armSubsystem, -1),
      new AutoElevator(elevatorsubsystem, -1),
      new DriveMeters(m_robotDrive, -400),
      new RobotStateChanger(ledSubsystem, 1));

      case 5:
      return
      new SequentialCommandGroup(new AutoElevator(elevatorsubsystem, 2),
      new AutoArm(armSubsystem, 1),
      new ClawSet(clawSubsystem).withTimeout(0.5),
      new AutoArm(armSubsystem, -1),
      new AutoElevator(elevatorsubsystem, -1),
      new Drive_PitchControl(m_robotDrive, false),
      new AutobalanceCommand(m_robotDrive));

      case 6:
      return
      new SequentialCommandGroup(new AutoElevator(elevatorsubsystem, 2),
      new AutoArm(armSubsystem, 1),
      new ClawSet(clawSubsystem).withTimeout(0.5),
      new AutoArm(armSubsystem, -1),
      new AutoElevator(elevatorsubsystem, -1),
      new RobotStateChanger(ledSubsystem, 1));

      default:
        return new RobotStateChanger(ledSubsystem, 2);
    }

  }

  public Command getAutonomousCommand() {
    return null;
  }
}
