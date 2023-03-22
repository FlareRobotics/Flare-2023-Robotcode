package frc.robot.commands.Auto;

import java.util.HashMap;
import java.util.List;
import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.auto.PIDConstants;
import com.pathplanner.lib.auto.RamseteAutoBuilder;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.AutoConstants;
import frc.robot.commands.Arm.AutoArm;
import frc.robot.commands.Claw.ClawSet;
import frc.robot.commands.Elevator.AutoElevator;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ClawSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;

public class FlareTrajectory extends CommandBase {

    private static DriveSubsystem m_robotDrive;
    private ElevatorSubsystem elevatorsubsystem;
    private ArmSubsystem armSubsystem;
    private ClawSubsystem clawSubsystem;

    public FlareTrajectory(DriveSubsystem driveSubsystem, ElevatorSubsystem elevatorSubsystem, ArmSubsystem armSubsystem, ClawSubsystem clawSubsystem) {
        addRequirements(driveSubsystem);
        this.elevatorsubsystem = elevatorSubsystem;
        this.armSubsystem = armSubsystem;
        this.clawSubsystem = clawSubsystem;

        m_robotDrive = driveSubsystem;
    }

    public Command getAutonomousCommand() {
        HashMap<String, Command> mainPathEvents = new HashMap<>();
        List<PathPlannerTrajectory> examplePath = PathPlanner.loadPathGroup("Main Path", new PathConstraints(4, 3));

        mainPathEvents.put("Start", new SequentialCommandGroup(new AutoElevator(elevatorsubsystem, 3),
            new AutoArm(armSubsystem, 3)));
        mainPathEvents.put("Pick", new ClawSet(clawSubsystem));
        mainPathEvents.put("Put", new SequentialCommandGroup(new AutoElevator(elevatorsubsystem, 2),
            new AutoArm(armSubsystem, 2)));
        RamseteAutoBuilder autoBuilder = new RamseteAutoBuilder(
                m_robotDrive::getPose,
                m_robotDrive::resetOdometry,
                new RamseteController(AutoConstants.kRamseteB, AutoConstants.kRamseteZeta),
                AutoConstants.kDriveKinematics,
                new SimpleMotorFeedforward(AutoConstants.ksVolts, AutoConstants.kvVoltSecondsPerMeter),
                m_robotDrive::getWheelSpeeds,
                new PIDConstants(5.0, 0, 0),
                m_robotDrive::tankDriveVolts,
                mainPathEvents,
                true,
                m_robotDrive);

        return autoBuilder.fullAuto(examplePath);
    }
}
