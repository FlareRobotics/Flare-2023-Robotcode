package frc.robot;

import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.util.Units;

public final class Constants {

    public static class ElevatorConstants {
        public static int elevator_motor_port = 0;
        public static double elevator_gear_ratio = 20d;

        public static double elevator_hiz = 0.5d;

        public static double elevator_distance_per_rotation = 4.5d;

        // All in cm
        public static int bottom_row_height = 15;
        public static int middle_row_height = 87;
        public static int top_row_height = 117;
        public static int substation_height = 95;
    }

    public static class AlignConstants {
        // All in CM
        public static double outermost_cone_distance = 46.5d;
        public static double substation_distance = 59d;
    }

    public static class ArmConstants {
        public static int arm_motor_port = 1;

        public static double arm_gear_ratio = 20d;

        public static double arm_hiz = 0.5d;

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

        public static final double ksVolts = 0.22;
        public static final double kvVoltSecondsPerMeter = 1.98;
        public static final double kaVoltSecondsSquaredPerMeter = 0.2;
        public static final double kTrackwidthMeters = 0.69;
        // Example value only - as above, this must be tuned for your drive!
        public static final double kPDriveVel = 8.5;

        public static final DifferentialDriveKinematics kDriveKinematics = new DifferentialDriveKinematics(
                kTrackwidthMeters);

    }

    public static final class FlrvsnConstants {

        public static final double CAMERA_HEIGHT_METERS = 0.92d;
        public static final double TARGET_HEIGHT_METERS = 2.55d;
        public static final String Camera_Name = "Camera";
        public static final double Camera_Pitch = 60;
        // The yaw offset when straight
        public static final double YAW_OFFSET = 3;
        // How far can the robot be from a target? (deg)
        public static final double YAW_TOLERANCE = 3d;

    }

    public static final class IntakeConstants {

        public static final int sol_intake_motor_port = 0;
        public static final int sag_intake_motor_port = 0;

        public static final int compressor_port = 0;

        public static final int sol_intake_solenoid_port1 = 6;
        public static final int sol_intake_solenoid_port2 = 1;

        public static final int sag_intake_solenoid_port1 = 7;
        public static final int sag_intake_solenoid_port2 = 0;

        public static final double intake_hiz = 1;

        public static final int sol_intake_duz_buton_id = 0;
        public static final int sol_intake_terse_buton_id = 0;
        public static final int sag_intake_duz_buton_id = 0;
        public static final int sag_intake_terse_buton_id = 0;

    }

    public static final class PigeonConstants {
        public static final int pigeon_port = 0;
    }

    public static final int kSlotIdx = 0;

    public static final int kPIDLoopIdx = 0;

    public static final int kTimeoutMs = 30;

    public static final class AutoConstants {
        public static final double kMaxSpeedMetersPerSecond = 3;
        public static final double kMaxAccelerationMetersPerSecondSquared = 1;

        public static final double kRamseteB = 2;
        public static final double kRamseteZeta = 0.7;

        // Auto Balance pitch axis tolerance
        public static final double pitch_tolerance = 0.5;

    }
}