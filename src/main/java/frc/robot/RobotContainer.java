package frc.robot;

import java.util.HashMap;
import java.util.List;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Custom.DPadButton;
import frc.robot.Custom.ResetRobot;
import frc.robot.Custom.DPadButton.Direction;
import frc.robot.Custom.RobotState;
import frc.robot.Custom.SupplyGather;
import frc.robot.commands.Align.AlignForCube;
import frc.robot.commands.Align.AlignForLeft;
import frc.robot.commands.Align.AlignForLeft_2;
import frc.robot.commands.Align.AlignForMidLeft;
import frc.robot.commands.Align.AlignForMidRight;
import frc.robot.commands.Align.AlignForRight;
import frc.robot.commands.Align.AlignForRight_2;
import frc.robot.commands.Align.AlignForSubstation;
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

  public static XboxController driver_main = new XboxController(0);
  public static XboxController driver_2 = new XboxController(1);
  private static final ElevatorSubsystem elevatorsubsystem = new ElevatorSubsystem();
  private static final ClawSubsystem clawSubsystem = new ClawSubsystem();
  private static final ArmSubsystem armSubsystem = new ArmSubsystem();
  private static final DriveSubsystem m_robotDrive = new DriveSubsystem();
  public final LedSubsystem ledSubsystem = new LedSubsystem();

  public static SendableChooser<Integer> autoChooser = new SendableChooser<>();

  public static boolean clawOpen = false;
  public static RobotState currentState = RobotState.None;

  public RobotContainer() {
    configureButtonBindings();
    autoChooser.setDefaultOption("TAXI", 0);
    autoChooser.addOption("1 Piece Forward + TAXI", 1);
    autoChooser.addOption("1 Piece Forward + Auto Balance", 2);
    autoChooser.addOption("Auto Balance", 3);
    SmartDashboard.putData(autoChooser);

    ledSubsystem.setDefaultCommand(new LedController(ledSubsystem, RobotContainer.currentState));
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
    /*
     * // Claw For Cone
     * new JoystickButton(driver_main, XboxController.Button.kRightBumper.value)
     * .toggleOnTrue(new ClawSet(clawSubsystem, !clawOpen, true));
     * 
     * // Claw For Cube
     * new JoystickButton(driver_main, XboxController.Button.kRightBumper.value)
     * .toggleOnTrue(new ClawSet(clawSubsystem, !clawOpen, false));
     * 
     * // Align For Left
     * new DPadButton(driver_2, Direction.LEFT).whileTrue(new ParallelCommandGroup(
     * new AlignForLeft(m_robotDrive),
     * new AlignForRight_2(m_robotDrive),
     * new AlignForMidLeft(m_robotDrive)));
     * 
     * // Align For Right
     * new DPadButton(driver_2, Direction.RIGHT).whileTrue(new ParallelCommandGroup(
     * new AlignForLeft_2(m_robotDrive),
     * new AlignForRight(m_robotDrive),
     * new AlignForMidRight(m_robotDrive)));
     * 
     * // Align For Cube
     * new DPadButton(driver_2, Direction.UP).whileTrue(new
     * AlignForCube(m_robotDrive));
     * 
     * // Align For Substation + move arm and elevator
     * new DPadButton(driver_2, Direction.DOWN).whileTrue(new
     * SequentialCommandGroup(new AlignForSubstation(m_robotDrive), new
     * AutoElevator(elevatorsubsystem, 4),
     * new AutoArm(armSubsystem, 4)));
     * 
     * // Move Arm and Elevator
     * // BOTTOM ROW
     * new JoystickButton(driver_2, XboxController.Button.kA.value)
     * .whileTrue(new SequentialCommandGroup(new AutoElevator(elevatorsubsystem, 1),
     * new AutoArm(armSubsystem, 1)));
     * 
     * // Move Arm and Elevator
     * // Middle ROW
     * new JoystickButton(driver_2, XboxController.Button.kX.value)
     * .whileTrue(new SequentialCommandGroup(new AutoElevator(elevatorsubsystem, 2),
     * new AutoArm(armSubsystem, 2)));
     * 
     * // Move Arm and Elevator
     * // HIGH ROW
     * new JoystickButton(driver_2, XboxController.Button.kY.value)
     * .whileTrue(new SequentialCommandGroup(new AutoElevator(elevatorsubsystem, 3),
     * new AutoArm(armSubsystem, 3)));
     * 
     */ // Supply Gather (Cube or Cone)
    new JoystickButton(driver_main, XboxController.Button.kLeftBumper.value)
        .whileTrue(new SupplyGather(ledSubsystem));

    // Reset robot
    // new JoystickButton(driver_2, XboxController.Button.kRightBumper.value)
    // .toggleOnTrue(new ResetRobot(armSubsystem, elevatorsubsystem,
    // clawSubsystem));
  }

  public static HashMap<String, Command> mainPathEvents = new HashMap<>();

  public static Command getAuto() {
    switch (autoChooser.getSelected()) {
      case 0:
        return null;
      case 1:
        return null;
      case 2:
        // Flare Trajectory Example
        List<PathPlannerTrajectory> examplePath = PathPlanner.loadPathGroup("Main Path", new PathConstraints(4, 3));

        mainPathEvents.put("Start", new SequentialCommandGroup(new AutoElevator(elevatorsubsystem, 3),
            new AutoArm(armSubsystem, 3)));
        mainPathEvents.put("Pick", new ClawSet(clawSubsystem, true, false));
        mainPathEvents.put("Put", new SequentialCommandGroup(new AutoElevator(elevatorsubsystem, 2),
            new AutoArm(armSubsystem, 2)));

        return new FlareTrajectory(m_robotDrive, examplePath, mainPathEvents);
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