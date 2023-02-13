package frc.robot;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Custom.DPadButton;
import frc.robot.Custom.DPadButton.Direction;
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
import frc.robot.commands.Claw.ClawSet;
import frc.robot.commands.Claw.ToggleCompressor;
import frc.robot.commands.Drive.JoystickDriveCommand;
import frc.robot.commands.Elevator.AutoElevator;
import frc.robot.commands.Elevator.ManuelElevator;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ClawSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

//8054 <3
public class RobotContainer {

  public static XboxController driver_main = new XboxController(0);
  public static XboxController driver_2 = new XboxController(1);
  private final ElevatorSubsystem elevatorsubsystem = new ElevatorSubsystem();
  private final ClawSubsystem clawSubsystem = new ClawSubsystem();
  private final ArmSubsystem armSubsystem = new ArmSubsystem();
  private final DriveSubsystem m_robotDrive = new DriveSubsystem();

  public static SendableChooser<Integer> autoChooser = new SendableChooser<>();

  public RobotContainer() {
    configureButtonBindings();
    autoChooser.setDefaultOption("TAXI", 0);
    autoChooser.addOption("1 Piece Forward + TAXI", 1);
    autoChooser.addOption("1 Piece Forward + Auto Balance", 2);
    autoChooser.addOption("Auto Balance", 3);
    SmartDashboard.putData(autoChooser);
    m_robotDrive.setDefaultCommand(
        new JoystickDriveCommand(
            m_robotDrive,
            () -> -driver_main.getRawAxis(4),
            () -> driver_main.getRawAxis(1)));
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

    // Claw Open
    new JoystickButton(driver_main, XboxController.Button.kRightBumper.value)
        .toggleOnTrue(new ClawSet(clawSubsystem, true));
    // Claw Close
    new JoystickButton(driver_main, XboxController.Button.kLeftBumper.value)
        .toggleOnTrue(new ClawSet(clawSubsystem, false));

    // Align For Left
    new DPadButton(driver_2, Direction.LEFT).whileTrue(new ParallelCommandGroup(
        new AlignForLeft(m_robotDrive),
        new AlignForRight_2(m_robotDrive),
        new AlignForMidLeft(m_robotDrive)));
    // Align For Right
    new DPadButton(driver_2, Direction.RIGHT).whileTrue(new ParallelCommandGroup(
        new AlignForLeft_2(m_robotDrive),
        new AlignForRight(m_robotDrive),
        new AlignForMidRight(m_robotDrive)));

    // Align For Cube
    new DPadButton(driver_2, Direction.UP).whileTrue(new AlignForCube(m_robotDrive));

    // Align For Substation + move arm and elevator
    new DPadButton(driver_2, Direction.DOWN).whileTrue(new SequentialCommandGroup(new AlignForSubstation(m_robotDrive), new AutoElevator(elevatorsubsystem, 4), 
    new AutoArm(armSubsystem, 4)));

    // Move Arm and Elevator
    // BOTTOM ROW
    new JoystickButton(driver_2, XboxController.Button.kA.value)
        .whileTrue(new SequentialCommandGroup(new AutoElevator(elevatorsubsystem, 1), 
            new AutoArm(armSubsystem, 1)));

    // Move Arm and Elevator
    // Middle ROW
    new JoystickButton(driver_2, XboxController.Button.kX.value)
        .whileTrue(new SequentialCommandGroup(new AutoElevator(elevatorsubsystem, 2), 
            new AutoArm(armSubsystem, 2)));

    // Move Arm and Elevator
    // HIGH ROW
    new JoystickButton(driver_2, XboxController.Button.kY.value)
        .whileTrue(new SequentialCommandGroup(new AutoElevator(elevatorsubsystem, 3), 
            new AutoArm(armSubsystem, 3)));
  }

  public static Command getAuto() {
    switch (autoChooser.getSelected()) {
      case 0:
        return null;
      case 1:
        return null;
      case 2:
        return null;
      case 3:
        return null;
      default:
        return null;
    }
  }

  public Command getAutonomousCommand() {
    return getAuto();

  }
}

/*
 * 
 * import java.util.List;
 * 
 * import edu.wpi.first.math.controller.PIDController;
 * import edu.wpi.first.math.controller.RamseteController;
 * import edu.wpi.first.math.controller.SimpleMotorFeedforward;
 * import edu.wpi.first.math.geometry.Pose2d;
 * import edu.wpi.first.math.geometry.Rotation2d;
 * import edu.wpi.first.math.geometry.Translation2d;
 * import edu.wpi.first.math.trajectory.Trajectory;
 * import edu.wpi.first.math.trajectory.TrajectoryConfig;
 * import edu.wpi.first.math.trajectory.TrajectoryGenerator;
 * import
 * edu.wpi.first.math.trajectory.constraint.DifferentialDriveVoltageConstraint;
 * 
 * import edu.wpi.first.wpilibj2.command.RamseteCommand;
 * 
 * import frc.robot.Constants.AutoConstants;
 * import frc.robot.Constants.DriveConstants;
 * 
 * // Create a voltage constraint to ensure we don't accelerate too fast
 * var autoVoltageConstraint =
 * new DifferentialDriveVoltageConstraint(
 * new SimpleMotorFeedforward(
 * DriveConstants.ksVolts,
 * DriveConstants.kvVoltSecondsPerMeter,
 * DriveConstants.kaVoltSecondsSquaredPerMeter),
 * DriveConstants.kDriveKinematics,
 * 10);
 * 
 * // Create config for trajectory
 * TrajectoryConfig config =
 * new TrajectoryConfig(
 * AutoConstants.kMaxSpeedMetersPerSecond,
 * AutoConstants.kMaxAccelerationMetersPerSecondSquared)
 * // Add kinematics to ensure max speed is actually obeyed
 * .setKinematics(DriveConstants.kDriveKinematics)
 * // Apply the voltage constraint
 * .addConstraint(autoVoltageConstraint);
 * 
 * // An example trajectory to follow. All units in meters.
 * Trajectory exampleTrajectory =
 * TrajectoryGenerator.generateTrajectory(
 * // Start at the origin facing the +X direction
 * new Pose2d(0, 0, new Rotation2d(0)),
 * // Pass through these two interior waypoints, making an 's' curve path
 * List.of(new Translation2d(1, 1), new Translation2d(2, -1)),
 * // End 3 meters straight ahead of where we started, facing forward
 * new Pose2d(3, 0, new Rotation2d(0)),
 * // Pass config
 * config);
 * 
 * RamseteCommand ramseteCommand =
 * new RamseteCommand(
 * exampleTrajectory,
 * m_robotDrive::getPose,
 * new RamseteController(AutoConstants.kRamseteB, AutoConstants.kRamseteZeta),
 * new SimpleMotorFeedforward(
 * DriveConstants.ksVolts,
 * DriveConstants.kvVoltSecondsPerMeter,
 * DriveConstants.kaVoltSecondsSquaredPerMeter),
 * DriveConstants.kDriveKinematics,
 * m_robotDrive::getWheelSpeeds,
 * new PIDController(DriveConstants.kPDriveVel, 0, 0),
 * new PIDController(DriveConstants.kPDriveVel, 0, 0),
 * // RamseteCommand passes volts to the callback
 * m_robotDrive::tankDriveVolts,
 * m_robotDrive);
 * 
 * // Reset odometry to the starting pose of the trajectory.
 * m_robotDrive.resetOdometry(exampleTrajectory.getInitialPose());
 * 
 * // Run path following command, then stop at the end.
 * return ramseteCommand.andThen(() -> m_robotDrive.tankDriveVolts(0, 0));
 * }
 */