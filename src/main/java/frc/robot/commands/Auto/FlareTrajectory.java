package frc.robot.commands.Auto;

import java.util.HashMap;
import java.util.List;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.auto.PIDConstants;
import com.pathplanner.lib.auto.RamseteAutoBuilder;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.AutoConstants;
import frc.robot.subsystems.DriveSubsystem;

public class FlareTrajectory extends CommandBase {

    private static DriveSubsystem m_robotDrive;
    private static List<PathPlannerTrajectory> trajectory;
    private static HashMap<String, Command> theEventMap;

    public FlareTrajectory(DriveSubsystem driveSubsystem, List<PathPlannerTrajectory> traj,
            HashMap<String, Command> eventMap) {
        addRequirements(driveSubsystem);
        m_robotDrive = driveSubsystem;
        trajectory = traj;
        theEventMap = eventMap;
    }

    public Command getAutonomousCommand() {
        RamseteAutoBuilder autoBuilder = new RamseteAutoBuilder(
                m_robotDrive::getPose,
                m_robotDrive::resetOdometry,
                new RamseteController(AutoConstants.kRamseteB, AutoConstants.kRamseteZeta),
                AutoConstants.kDriveKinematics,
                new SimpleMotorFeedforward(AutoConstants.ksVolts, AutoConstants.kvVoltSecondsPerMeter),
                m_robotDrive::getWheelSpeeds,
                new PIDConstants(5.0, 0, 0),
                m_robotDrive::tankDriveVolts,
                theEventMap,
                true,
                m_robotDrive);

        return autoBuilder.fullAuto(trajectory);
    }
}
