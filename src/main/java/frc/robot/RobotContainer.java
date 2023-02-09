package frc.robot;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.commands.Arm.ManuelArm;
import frc.robot.commands.Claw.ClawSet;
import frc.robot.commands.Claw.ToggleCompressor;
import frc.robot.commands.Drive.JoystickDriveCommand;
import frc.robot.commands.Elevator.ManuelElevator;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ClawSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.FlareVisionSubsystem;
import edu.wpi.first.wpilibj2.command.Command;

import edu.wpi.first.wpilibj2.command.button.JoystickButton;

public class RobotContainer {

  public static XboxController driver_joy = new XboxController(0);
  // private final FlareVisionSubsystem flareVisionSubsystem = new
  // FlareVisionSubsystem();
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
            () -> -driver_joy.getRawAxis(4),
            () -> driver_joy.getRawAxis(1)));
  }

  private void configureButtonBindings() {
<<<<<<< Updated upstream
    //  Manuel Elevator 
    new JoystickButton(driver_joy,XboxController.Button.kA.value).whileActiveOnce(new ManuelElevator(elevatorsubsystem, true));
    new JoystickButton(driver_joy,XboxController.Button.kY.value).whileActiveOnce(new ManuelElevator(elevatorsubsystem, false));
    
    // Manuel Arm 
    new JoystickButton(driver_joy, XboxController.Button.kX.value).whileActiveOnce(new ManuelArm(armSubsystem,true));
    new JoystickButton(driver_joy, XboxController.Button.kB.value).whileActiveOnce(new ManuelArm(armSubsystem,false));

    // Compressor Toggle
    new JoystickButton(driver_joy,XboxController.Button.kStart.value).toggleWhenPressed(new ToggleCompressor(clawSubsystem));

    //Claw Open
    new JoystickButton(driver_joy, XboxController.Button.kRightBumper.value).whileActiveOnce(new ClawSet(clawSubsystem,true));
    //Claw Close
    new JoystickButton(driver_joy, XboxController.Button.kLeftBumper.value).whileActiveOnce(new ClawSet(clawSubsystem,false));
  
}
=======
    // Manuel Elevator
    new JoystickButton(driver_joy, XboxController.Button.kA.value)
        .whileTrue(new ManuelElevator(elevatorsubsystem, true));
    new JoystickButton(driver_joy, XboxController.Button.kA.value)
        .whileTrue(new ManuelElevator(elevatorsubsystem, true));
    new JoystickButton(driver_joy, XboxController.Button.kY.value)
        .whileTrue(new ManuelElevator(elevatorsubsystem, false));

    // Manuel Arm
    new JoystickButton(driver_joy, XboxController.Button.kX.value).whileTrue(new ManuelArm(armSubsystem, true));
    new JoystickButton(driver_joy, XboxController.Button.kB.value).whileTrue(new ManuelArm(armSubsystem, false));

    // Compressor Toggle
    new JoystickButton(driver_joy, XboxController.Button.kStart.value)
        .toggleOnTrue(new ToggleCompressor(clawSubsystem));

    // Claw Open
    new JoystickButton(driver_joy, XboxController.Button.kRightBumper.value)
        .toggleOnTrue(new ClawSet(clawSubsystem, true));
    // Claw Close
    new JoystickButton(driver_joy, XboxController.Button.kLeftBumper.value)
        .toggleOnTrue(new ClawSet(clawSubsystem, false));
>>>>>>> Stashed changes

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
