package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.PID.PidConstants;

public class ElevatorSubsystem extends SubsystemBase {
        public static WPI_TalonFX elevator_motor = new WPI_TalonFX(ElevatorConstants.elevator_motor_port);

        public ElevatorSubsystem() {
                /* Factory default hardware to prevent unexpected behavior */
                elevator_motor.configFactoryDefault();

                /* Configure Sensor Source for Pirmary PID */
                elevator_motor.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor,
                                PidConstants.TurretConstants.kPIDLoopIdx,
                                PidConstants.TurretConstants.kTimeoutMs);

                /*
                 * set deadband to super small 0.001 (0.1 %).
                 * The default deadband is 0.04 (4 %)
                 */
                elevator_motor.configNeutralDeadband(0.001, PidConstants.TurretConstants.kTimeoutMs);

                /**
                 * Configure Talon FX Output and Sensor direction accordingly Invert Motor to
                 * have green LEDs when driving Talon Forward / Requesting Postiive Output Phase
                 * sensor to have positive increment when driving Talon Forward (Green LED)
                 */
                elevator_motor.setSensorPhase(false);
                elevator_motor.setInverted(false);
                /*
                 * Talon FX does not need sensor phase set for its integrated sensor
                 * This is because it will always be correct if the selected feedback device is
                 * integrated sensor (default value)
                 * and the user calls getSelectedSensor* to get the sensor's position/velocity.
                 * 
                 * https://phoenix-documentation.readthedocs.io/en/latest/ch14_MCSensor.html#
                 * sensor-phase
                 */
                // elevator_motor.setSensorPhase(true);

                /* Set relevant frame periods to be at least as fast as periodic rate */
                elevator_motor.setStatusFramePeriod(StatusFrameEnhanced.Status_13_Base_PIDF0, 10,
                                PidConstants.TurretConstants.kTimeoutMs);
                elevator_motor.setStatusFramePeriod(StatusFrameEnhanced.Status_10_MotionMagic, 10,
                                PidConstants.TurretConstants.kTimeoutMs);

                /* Set the peak and nominal outputs */
                elevator_motor.configNominalOutputForward(0, PidConstants.TurretConstants.kTimeoutMs);
                elevator_motor.configNominalOutputReverse(0, PidConstants.TurretConstants.kTimeoutMs);
                elevator_motor.configPeakOutputForward(PidConstants.TurretConstants.kGains.kPeakOutput,
                                PidConstants.TurretConstants.kTimeoutMs);
                elevator_motor.configPeakOutputReverse(-PidConstants.TurretConstants.kGains.kPeakOutput,
                                PidConstants.TurretConstants.kTimeoutMs);

                /* Set Motion Magic gains in slot0 - see documentation */
                elevator_motor.selectProfileSlot(PidConstants.TurretConstants.kSlotIdx,
                                PidConstants.TurretConstants.kPIDLoopIdx);
                elevator_motor.config_kF(PidConstants.TurretConstants.kSlotIdx, PidConstants.TurretConstants.kGains.kF,
                                PidConstants.TurretConstants.kTimeoutMs);
                elevator_motor.config_kP(PidConstants.TurretConstants.kSlotIdx, 29.5,
                                PidConstants.TurretConstants.kTimeoutMs);
                elevator_motor.config_kI(PidConstants.TurretConstants.kSlotIdx, PidConstants.TurretConstants.kGains.kI,
                                PidConstants.TurretConstants.kTimeoutMs);
                elevator_motor.config_kD(PidConstants.TurretConstants.kSlotIdx, 0,
                                PidConstants.TurretConstants.kTimeoutMs);

                /* Set acceleration and vcruise velocity - see documentation */
                elevator_motor.configMotionCruiseVelocity(3000, PidConstants.TurretConstants.kTimeoutMs);
                elevator_motor.configMotionAcceleration(4000, PidConstants.TurretConstants.kTimeoutMs);

                /* Zero the sensor once on robot boot up */
                elevator_motor.setSelectedSensorPosition(0, PidConstants.TurretConstants.kPIDLoopIdx,
                                PidConstants.TurretConstants.kTimeoutMs);

                elevator_motor.setNeutralMode(NeutralMode.Coast);

                elevator_motor.configMotionSCurveStrength(25);

                elevator_motor.configForwardSoftLimitThreshold(Constants.ElevatorConstants.elevator_forward_limit);
                elevator_motor.configReverseSoftLimitThreshold(0);

                elevator_motor.configForwardSoftLimitEnable(true, 0);
              //  elevator_motor.configReverseSoftLimitEnable(true, 0);
        }

        @Override
        public void periodic() {
                SmartDashboard.putNumber("ElevatorDistance", elevator_motor.getSelectedSensorPosition());
        }

        public void reset_encoder() {
                elevator_motor.setSelectedSensorPosition(0);
        }

        public static void elevator_motor_set(double hiz) {
                elevator_motor.set(hiz);
        }

        // Units to cm
        public static int elevator_yukseklik_cm() {
                return (int) (Math.round(elevator_motor.getSelectedSensorPosition() / 2048
                                / ElevatorConstants.elevator_gear_ratio
                                * ElevatorConstants.elevator_distance_per_rotation));
        }

        // cm to Units
        public static int elevator_yukseklik_units(int cm) {
                return (int) (2048 * ElevatorConstants.elevator_gear_ratio * cm
                                / ElevatorConstants.elevator_distance_per_rotation);
        }
}
