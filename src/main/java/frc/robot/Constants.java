package frc.robot;

import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.util.Units;

public final class Constants {

    public static class ElevatorConstants {
        public static int elevator_motor_port = 0;
        public static double elevator_gear_ratio = 20d;

        public static double elevator_speed = 0.5d;

        public static double elevator_distance_per_rotation = 4.5d;

        // All in cm
        public static int bottom_row_height = 15;
        public static int middle_row_height = 87;
        public static int top_row_height = 117;
        public static int substation_height = 95;
    }

    public static class AlignConstants {
        // All in CM
        public static double cone_distance = 46.5d;
        public static double substation_distance = 59d;
    }

    public static class ArmConstants {
        public static int arm_motor_port = 1;

        public static double arm_gear_ratio = 20d;

        public static double arm_speed = 0.5d;

        public static double arm_distance_per_rotation = 4.5d;

        // All in cm
        public static int bottom_row_distance = 10;
        public static int middle_row_distance = 60;
        public static int top_row_distance = 103;
        public static int substation_distance = 17;
    }

    public static class ClawConstants {
        public static int claw_solenoid_port_ileri = 0;
        public static int claw_solenoid_port_geri = 1;
    }

    public static class DriveConstants {
        public static int sagon_falcon_port = 8;
        public static int sagarka_falcon_port = 9;
        public static int solon_falcon_port = 3;
        public static int solarka_falcon_port = 4;

        public static double drive_disli_orani = (34 / 18 * 5);

        public static int kEncoderCPR = 2048;
        public static double kWheelDiameterMeters = Units.inchesToMeters(6);
    }

    public static final class VisionConstants {
        public static final double CAMERA_HEIGHT_METERS = 0.92d;
        public static final double TARGET_HEIGHT_METERS = 2.55d;
        public static final String Camera_Name = "Camera";
    }

    public static final class PigeonConstants {
        public static final int pigeon_port = 0;
    }

    public static final class LedConstans {
        public static final int led_port = 0;
        public static final int led_uzunluk = 30;
    }

    public static final int kSlotIdx = 0;

    /**
     * Talon FX supports multiple (cascaded) PID loops. For
     * now we just want the primary one.
     */
    public static final int kPIDLoopIdx = 0;

    /**
     * Set to zero to skip waiting for confirmation, set to nonzero to wait and
     * report to DS if action fails.
     */
    public static final int kTimeoutMs = 30;

    public static final class AutoConstants {
        // These are example values only - DO NOT USE THESE FOR YOUR OWN ROBOT!
        // These characterization values MUST be determined either experimentally or
        // theoretically
        // for *your* robot's drive.
        // The Robot Characterization Toolsuite provides a convenient tool for obtaining
        // these
        // values for your robot.
        public static final double ksVolts = 0.22;
        public static final double kvVoltSecondsPerMeter = 1.98;
        public static final double kaVoltSecondsSquaredPerMeter = 0.2;

        // Example value only - as above, this must be tuned for your drive!
        public static final double kPDriveVel = 8.5;

        public static final double kMaxSpeedMetersPerSecond = 3;
        public static final double kMaxAccelerationMetersPerSecondSquared = 1;

        // Reasonable baseline values for a RAMSETE follower in units of meters and
        // seconds
        public static final double kRamseteB = 2;
        public static final double kRamseteZeta = 0.7;

        public static final double kTrackwidthMeters = 0.69;
        public static final DifferentialDriveKinematics kDriveKinematics = new DifferentialDriveKinematics(
                kTrackwidthMeters);

        // Auto Balance pitch axis tolerance
        public static final double pitch_tolerance = 0.5;
    }
}