package frc.robot.PID;

public class PidConstants {

	public static class ElevatorConstants {
		public static final int kSlotIdx = 0;

		public static final int kPIDLoopIdx = 0;

		
		public static final int kTimeoutMs = 30;

		/**
		 * Gains used in Motion Magic, to be adjusted accordingly
		 * Gains(kp, ki, kd, kf, izone, peak output);
		 */
		public static final Gains kGains = new Gains(1, 0.0, 0.5, 0.2, 0, 2d);
	}

	public static class ArmConstants {
		public static final int kSlotIdx = 0;

		public static final int kPIDLoopIdx = 0;

		
		public static final int kTimeoutMs = 30;

		/**
		 * Gains used in Motion Magic, to be adjusted accordingly
		 * Gains(kp, ki, kd, kf, izone, peak output);
		 */
		public static final Gains kGains = new Gains(1, 0.0, 0.5, 0.2, 0, 2d);
	}

	public static class DriveConstants {
		public final static int kSensorUnitsPerRotation = 2048;

		public final static double kRotationsToTravel = 6;


		public final static double kTurnTravelUnitsPerRotation = 3600;

		public final static int kTimeoutMs = 30;

		public final static double kNeutralDeadband = 0.001;

		public final static Gains kGains_Distanc = new Gains(0.1, 0.0, 0.0, 0.0, 100, 0.50);
		public final static Gains kGains_Turning = new Gains(2.0, 0.0, 4.0, 0.0, 200, 1.00);
		public final static Gains kGains_Velocit = new Gains(0.1, 0.0, 20.0, 1023.0 / 6800.0, 300, 0.50);
		public final static Gains kGains_MotProf = new Gains(1.0, 0.0, 0.0, 1023.0 / 6800.0, 400, 1.00);

		public final static int REMOTE_0 = 0;
		public final static int REMOTE_1 = 1;
		public final static int PID_PRIMARY = 0;
		public final static int PID_TURN = 1;
		
		public final static int SLOT_0 = 0;
		public final static int SLOT_1 = 1;
		public final static int SLOT_2 = 2;
		public final static int SLOT_3 = 3;
		
		public final static int kSlot_Distanc = SLOT_0;
		public final static int kSlot_Turning = SLOT_1;
		public final static int kSlot_Velocit = SLOT_2;
		public final static int kSlot_MotProf = SLOT_3;
	}
}