/**
 * Simple class containing constants used throughout project
 */
package frc.robot.PID;

public class PidConstants {

	public static class TurretConstants {
		/**
		 * Which PID slot to pull gains from. Starting 2018, you can choose from
		 * 0,1,2 or 3. Only the first two (0,1) are visible in web-based
		 * configuration.
		 */
		public static final int kSlotIdx = 0;

		/**
		 * Talon FX supports multiple (cascaded) PID loops. For
		 * now we just want the primary one.
		 */
		public static final int kPIDLoopIdx = 0;

		/**
		 * set to zero to skip waiting for confirmation, set to nonzero to wait and
		 * report to DS if action fails.
		 */
		public static final int kTimeoutMs = 30;

		/**
		 * Gains used in Motion Magic, to be adjusted accordingly
		 * Gains(kp, ki, kd, kf, izone, peak output);
		 */
		public static final Gains kGains = new Gains(0.5, 0.005, 3, 0.2, 0, 50d);
	}

	public static class DriveConstants {
		/**
		 * How many sensor units per rotation.
		 * Using CTRE Magnetic Encoder.
		 * 
		 * @link https://github.com/CrossTheRoadElec/Phoenix-Documentation#what-are-the-units-of-my-sensor
		 */
		public final static int kSensorUnitsPerRotation = 2048;

		/**
		 * Number of rotations to drive when performing Distance Closed Loop
		 */
		public final static double kRotationsToTravel = 6;

		public static final int kSlotIdx = 0;

		/**
		 * Talon FX supports multiple (cascaded) PID loops. For
		 * now we just want the primary one.
		 */
		public static final int kPIDLoopIdx = 0;

		/**
		 * Using the configSelectedFeedbackCoefficient() function, scale units to 3600
		 * per rotation.
		 * This is nice as it keeps 0.1 degrees of resolution, and is fairly intuitive.
		 */
		public final static double kTurnTravelUnitsPerRotation = 3600;

		/**
		 * Set to zero to skip waiting for confirmation.
		 * Set to nonzero to wait and report to DS if action fails.
		 */
		public final static int kTimeoutMs = 30;

		/**
		 * Motor neutral dead-band, set to the minimum 0.1%.
		 */
		public final static double kNeutralDeadband = 0.001;

		/**
		 * PID Gains may have to be adjusted based on the responsiveness of control
		 * loop.
		 * kF: 1023 represents output value to Talon at 100%, 6800 represents Velocity
		 * units at 100% output
		 * Not all set of Gains are used in this project and may be removed as desired.
		 * 
		 * kP kI kD kF Iz PeakOut
		 */
		public static final Gains kGains = new Gains(0.5, 0.005, 3, 0.2, 0, 100);

		/** ---- Flat constants, you should not need to change these ---- */
		/*
		 * We allow either a 0 or 1 when selecting an ordinal for remote devices [You
		 * can have up to 2 devices assigned remotely to a talon/victor]
		 */
		public final static int REMOTE_0 = 0;
		public final static int REMOTE_1 = 1;
		/*
		 * We allow either a 0 or 1 when selecting a PID Index, where 0 is primary and 1
		 * is auxiliary
		 */
		public final static int PID_PRIMARY = 0;
		public final static int PID_TURN = 1;
		/*
		 * Firmware currently supports slots [0, 3] and can be used for either PID Set
		 */
		public final static int SLOT_0 = 0;
		public final static int SLOT_1 = 1;
		public final static int SLOT_2 = 2;
		public final static int SLOT_3 = 3;
		/* ---- Named slots, used to clarify code ---- */
		public final static int kSlot_Distanc = SLOT_0;
		public final static int kSlot_Turning = SLOT_1;
		public final static int kSlot_Velocit = SLOT_2;
		public final static int kSlot_MotProf = SLOT_3;
	}

	public static class ShooterConstants {
		/**
		 * Which PID slot to pull gains from. Starting 2018, you can choose from
		 * 0,1,2 or 3. Only the first two (0,1) are visible in web-based
		 * configuration.
		 */
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

		/**
		 * PID Gains may have to be adjusted based on the responsiveness of control
		 * loop.
		 * kF: 1023 represents output value to Talon at 100%, 20660 represents Velocity
		 * units at 100% output
		 * 
		 * kP kI kD kF Iz PeakOut
		 */
		public final static Gains kGains_Velocit = new Gains(0.1, 0.001, 5, 1023.0 / 20660.0, 300, 1.00);
	}
}