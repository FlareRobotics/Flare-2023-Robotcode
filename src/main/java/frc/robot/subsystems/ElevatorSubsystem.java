package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.PID.PidConstants;

public class ElevatorSubsystem extends SubsystemBase {
        public static WPI_TalonFX elevator_motor = new WPI_TalonFX(ElevatorConstants.elevator_motor_port);

        public ElevatorSubsystem() {
                elevator_motor.configFactoryDefault();

                elevator_motor.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor,
                                PidConstants.ElevatorConstants.kPIDLoopIdx,
                                PidConstants.ElevatorConstants.kTimeoutMs);

                elevator_motor.configNeutralDeadband(0.001, PidConstants.ElevatorConstants.kTimeoutMs);

                elevator_motor.setSensorPhase(false);
                elevator_motor.setInverted(false);

                elevator_motor.setStatusFramePeriod(StatusFrameEnhanced.Status_13_Base_PIDF0, 10,
                                PidConstants.ElevatorConstants.kTimeoutMs);
                elevator_motor.setStatusFramePeriod(StatusFrameEnhanced.Status_10_MotionMagic, 10,
                                PidConstants.ElevatorConstants.kTimeoutMs);

                elevator_motor.configNominalOutputForward(0, PidConstants.ElevatorConstants.kTimeoutMs);
                elevator_motor.configNominalOutputReverse(0, PidConstants.ElevatorConstants.kTimeoutMs);
                elevator_motor.configPeakOutputForward(PidConstants.ElevatorConstants.kGains.kPeakOutput,
                                PidConstants.ElevatorConstants.kTimeoutMs);
                elevator_motor.configPeakOutputReverse(-PidConstants.ElevatorConstants.kGains.kPeakOutput,
                                PidConstants.ElevatorConstants.kTimeoutMs);

                elevator_motor.selectProfileSlot(PidConstants.ElevatorConstants.kSlotIdx,
                                PidConstants.ElevatorConstants.kPIDLoopIdx);
                elevator_motor.config_kF(PidConstants.ElevatorConstants.kSlotIdx, PidConstants.ElevatorConstants.kGains.kF,
                                PidConstants.ElevatorConstants.kTimeoutMs);
                elevator_motor.config_kP(PidConstants.ElevatorConstants.kSlotIdx, PidConstants.ElevatorConstants.kGains.kP,
                                PidConstants.ElevatorConstants.kTimeoutMs);
                elevator_motor.config_kI(PidConstants.ElevatorConstants.kSlotIdx, PidConstants.ElevatorConstants.kGains.kI,
                                PidConstants.ElevatorConstants.kTimeoutMs);
                elevator_motor.config_kD(PidConstants.ElevatorConstants.kSlotIdx, PidConstants.ElevatorConstants.kGains.kD,
                                PidConstants.ElevatorConstants.kTimeoutMs);

                elevator_motor.configMotionCruiseVelocity(3000, PidConstants.ElevatorConstants.kTimeoutMs);
                elevator_motor.configMotionAcceleration(4000, PidConstants.ElevatorConstants.kTimeoutMs);

                elevator_motor.setSelectedSensorPosition(0, PidConstants.ElevatorConstants.kPIDLoopIdx,
                                PidConstants.ElevatorConstants.kTimeoutMs);

                elevator_motor.setNeutralMode(NeutralMode.Brake);

                elevator_motor.configMotionSCurveStrength(25);
        }

        @Override
        public void periodic() {

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