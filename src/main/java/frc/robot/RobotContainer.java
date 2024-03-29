package frc.robot;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Custom.Distance_State;
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
    public float turn_rate = 2.0f; 
    public RobotContainer() {
        configureButtonBindings();
        autoChooser.setDefaultOption("No Auto", 0);
        autoChooser.addOption("Mobility", 1);
        autoChooser.addOption("Balance", 2);
        autoChooser.addOption("Middle Cube", 4);
        autoChooser.addOption("Middle Cone + Mobility", 5);
        autoChooser.addOption("Middle Cube + Turn + Balance", 7);
        autoChooser.addOption("Middle Cube + Mobility", 8);
        SmartDashboard.putData(autoChooser);

        ledSubsystem.setDefaultCommand(new LedController(ledSubsystem));
        m_robotDrive.setDefaultCommand(
                new ParallelCommandGroup(new JoystickDriveCommand(
                        m_robotDrive,
                        () -> -driver_main.getLeftY() / 1.25,
                        () -> -driver_main.getRightX() / turn_rate)));
    }

    private void configureButtonBindings() {
        // Manuel Elevator
        new JoystickButton(driver_2, 8)
                .whileTrue(new ManuelElevator(elevatorsubsystem, true));
        new JoystickButton(driver_2, 7)
                .whileTrue(new ManuelElevator(elevatorsubsystem, false));

        // Manuel Arm
        new JoystickButton(driver_main, XboxController.Button.kB.value).whileTrue(new ManuelArm(armSubsystem, true));
        new JoystickButton(driver_main, XboxController.Button.kX.value).whileTrue(new ManuelArm(armSubsystem, false));

        // Compressor Toggle
        new JoystickButton(driver_main, XboxController.Button.kStart.value)
                .toggleOnTrue(new ToggleCompressor(clawSubsystem));

        // Claw For Cone
        new JoystickButton(driver_main, XboxController.Button.kRightBumper.value)
                .toggleOnTrue(new ClawSet(clawSubsystem));

        // Wanted Status
        new JoystickButton(driver_2, 5).toggleOnTrue(new SupplyGather(ledSubsystem));
        // Reset Encoders
        new JoystickButton(driver_2, 9).whileTrue(new ResetRobot(armSubsystem, elevatorsubsystem, clawSubsystem));
        // Home Elevator PID
        // new JoystickButton(driver_2, 3).whileTrue(new AutoElevator(elevatorsubsystem, Distance_State.Zero_All));

        // // Auto Elevator
        // new JoystickButton(driver_2, 2)
        //         .whileTrue(new AutoElevator(elevatorsubsystem, Distance_State.Middle_Cube_Elevator));

        // new JoystickButton(driver_2, 1)
        //         .whileTrue(new AutoElevator(elevatorsubsystem, Distance_State.Middle_Cone_Elevator));

        // Led Close
        new JoystickButton(driver_2, 11).whileTrue(new RobotStateChanger(0));

        // Substation Test
        new JoystickButton(driver_2, 6).toggleOnTrue(
                new ParallelCommandGroup(new AutoElevator(elevatorsubsystem, Distance_State.Substation_Elevator),
                        new SequentialCommandGroup(new WaitCommand(1d),
                                new AutoArm(armSubsystem, Distance_State.Substation_Arm))));

        new JoystickButton(driver_2, 1).toggleOnTrue(new ParallelCommandGroup(
                new AutoElevator(elevatorsubsystem, Distance_State.Middle_Cube_Elevator),
                new SequentialCommandGroup(
                        new WaitCommand(.5),
                        new AutoArm(armSubsystem, Distance_State.Middle_Cone_Arm))));

        new JoystickButton(driver_2, 4).toggleOnTrue(new ParallelCommandGroup(
                new AutoElevator(elevatorsubsystem, Distance_State.Middle_Cone_Elevator_Auto),
                new SequentialCommandGroup(
                        new WaitCommand(.5),
                        new AutoArm(armSubsystem, Distance_State.Middle_Cone_Arm))));

        new JoystickButton(driver_2, 2).toggleOnTrue(new ParallelCommandGroup(
                new AutoArm(armSubsystem, Distance_State.Zero_All),
                new SequentialCommandGroup(
                        new WaitCommand(.8),
                        new AutoElevator(elevatorsubsystem, Distance_State.Zero_All))));

        new JoystickButton(driver_2, 3).toggleOnTrue(new AutoArm(armSubsystem, Distance_State.Zero_All));

       
        new JoystickButton(driver_main, XboxController.Button.kLeftStick.value).whileTrue(new RunCommand(() -> turn_rate = 2.8f));
       new JoystickButton(driver_main, XboxController.Button.kRightStick.value).whileTrue(new RunCommand(() -> turn_rate = 2.0f));
       
    }

    public static Command getAuto() {
        switch (autoChooser.getSelected()) {
            case 0:
                return new RobotStateChanger(2);

            case 1:
                return new SequentialCommandGroup(
                        new DriveMeters(m_robotDrive, -390),
                        new RobotStateChanger(1));

            case 2:
                return new SequentialCommandGroup(
                        new Drive_PitchControl(m_robotDrive, true),
                        new AutobalanceCommand(m_robotDrive));

            case 3:
                return new SequentialCommandGroup(new DriveMeters(m_robotDrive, -390),
                        new Drive_PitchControl(m_robotDrive, true),
                        new AutobalanceCommand(m_robotDrive));

            case 4:
                return new SequentialCommandGroup(
                        new ParallelCommandGroup(
                                new AutoElevator(elevatorsubsystem, Distance_State.Middle_Cube_Elevator),
                                new SequentialCommandGroup(
                                        new WaitCommand(.5),
                                        new AutoArm(armSubsystem, Distance_State.Middle_Cone_Arm))),
                        new ClawSet(clawSubsystem).withTimeout(0.5d),
                        new AutoArm(armSubsystem, Distance_State.Zero_All),
                        new AutoElevator(elevatorsubsystem, Distance_State.Zero_All),
                        new RobotStateChanger(1));

            case 5:
                return new SequentialCommandGroup(
                        new ParallelCommandGroup(
                                new AutoElevator(elevatorsubsystem, Distance_State.Middle_Cone_Elevator_Auto),
                                new SequentialCommandGroup(
                                        new WaitCommand(1.1d),
                                        new AutoArm(armSubsystem, Distance_State.Middle_Cone_Arm))),
                        new ClawSet(clawSubsystem).withTimeout(0.5d),
                        new ParallelCommandGroup(
                                new AutoArm(armSubsystem, Distance_State.Zero_All),
                                new AutoElevator(elevatorsubsystem, Distance_State.Zero_All),
                                new SequentialCommandGroup(
                                        new WaitCommand(1.5),
                                        new DriveMeters(m_robotDrive, -390))),
                        new RobotStateChanger(1));

            case 6:
                return new SequentialCommandGroup(
                        new ParallelCommandGroup(
                                new AutoElevator(elevatorsubsystem, Distance_State.Middle_Cube_Elevator),
                                new SequentialCommandGroup(
                                        new WaitCommand(.5),
                                        new AutoArm(armSubsystem, Distance_State.Middle_Cube_Arm))),
                        new ClawSet(clawSubsystem).withTimeout(0.5d),
                        new ParallelCommandGroup(
                                new SequentialCommandGroup(new WaitCommand(0.5),
                                        new AutoElevator(elevatorsubsystem, Distance_State.Zero_All)),
                                new AutoArm(armSubsystem, Distance_State.Zero_All),
                        new SequentialCommandGroup(
                                new WaitCommand(1.5),
                                new DriveMeters(m_robotDrive, -390))),
                        new Drive_PitchControl(m_robotDrive, true),
                        new AutobalanceCommand(m_robotDrive));

                case 7:
                        return new SequentialCommandGroup(
                                new ParallelCommandGroup(
                                        new AutoElevator(elevatorsubsystem, Distance_State.Middle_Cube_Elevator),
                                        new SequentialCommandGroup(
                                                new WaitCommand(.5),
                                                new AutoArm(armSubsystem, Distance_State.Middle_Cube_Arm))),
                                new ClawSet(clawSubsystem).withTimeout(0.5d),
                                new ParallelCommandGroup(
                                        new SequentialCommandGroup(new WaitCommand(0.5),
                                                new AutoElevator(elevatorsubsystem, Distance_State.Zero_All)),
                                        new AutoArm(armSubsystem, Distance_State.Zero_All),
                                new SequentialCommandGroup(
                                        new WaitCommand(1.9d),
                                        new DriveMeters(m_robotDrive, -3),
                                        new TurnDegrees(m_robotDrive, 160))),
                                new Drive_PitchControl(m_robotDrive, true),
                                new AutobalanceCommand(m_robotDrive));

                        case 8:
                                return new SequentialCommandGroup(
                                        new ParallelCommandGroup(
                                                new AutoElevator(elevatorsubsystem, Distance_State.Middle_Cube_Elevator),
                                                new SequentialCommandGroup(
                                                        new WaitCommand(0.7d),
                                                        new AutoArm(armSubsystem, Distance_State.Middle_Cube_Arm))),
                                        new ClawSet(clawSubsystem).withTimeout(0.5d),
                                        new ParallelCommandGroup(
                                                new AutoArm(armSubsystem, Distance_State.Zero_All),
                                                new AutoElevator(elevatorsubsystem, Distance_State.Zero_All),
                                                new SequentialCommandGroup(
                                                        new WaitCommand(1.5),
                                                        new DriveMeters(m_robotDrive, -390))),
                                        new RobotStateChanger(1));

            default:
                return new RobotStateChanger(2);
        }
    }

    public Command getAutonomousCommand() {
        return new ResetRobot(armSubsystem, elevatorsubsystem, clawSubsystem).withTimeout(0.1)
                .andThen(
                        new ParallelCommandGroup(
                                getAuto(), new LedController(ledSubsystem)));
    }
}
