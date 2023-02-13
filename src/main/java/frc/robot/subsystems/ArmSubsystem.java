package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ArmConstants;
import frc.robot.PID.PidConstants;

public class ArmSubsystem extends SubsystemBase {
  public static WPI_TalonFX arm_motor = new WPI_TalonFX(ArmConstants.arm_motor_port);

  public ArmSubsystem() {

    arm_motor.configFactoryDefault();

    arm_motor.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor,
        PidConstants.ArmConstants.kPIDLoopIdx,
        PidConstants.ArmConstants.kTimeoutMs);

    arm_motor.configNeutralDeadband(0.001, PidConstants.ArmConstants.kTimeoutMs);

    arm_motor.setSensorPhase(false);
    arm_motor.setInverted(false);

    arm_motor.setStatusFramePeriod(StatusFrameEnhanced.Status_13_Base_PIDF0, 10,
        PidConstants.ArmConstants.kTimeoutMs);
    arm_motor.setStatusFramePeriod(StatusFrameEnhanced.Status_10_MotionMagic, 10,
        PidConstants.ArmConstants.kTimeoutMs);

    arm_motor.configNominalOutputForward(0, PidConstants.ArmConstants.kTimeoutMs);
    arm_motor.configNominalOutputReverse(0, PidConstants.ArmConstants.kTimeoutMs);
    arm_motor.configPeakOutputForward(PidConstants.ArmConstants.kGains.kPeakOutput,
        PidConstants.ArmConstants.kTimeoutMs);
    arm_motor.configPeakOutputReverse(-PidConstants.ArmConstants.kGains.kPeakOutput,
        PidConstants.ArmConstants.kTimeoutMs);

    arm_motor.selectProfileSlot(PidConstants.ArmConstants.kSlotIdx,
        PidConstants.ArmConstants.kPIDLoopIdx);
    arm_motor.config_kF(PidConstants.ArmConstants.kSlotIdx, PidConstants.ArmConstants.kGains.kF,
        PidConstants.ArmConstants.kTimeoutMs);
    arm_motor.config_kP(PidConstants.ArmConstants.kSlotIdx, PidConstants.ArmConstants.kGains.kP,
        PidConstants.ArmConstants.kTimeoutMs);
    arm_motor.config_kI(PidConstants.ArmConstants.kSlotIdx, PidConstants.ArmConstants.kGains.kI,
        PidConstants.ArmConstants.kTimeoutMs);
    arm_motor.config_kD(PidConstants.ArmConstants.kSlotIdx, PidConstants.ArmConstants.kGains.kD,
        PidConstants.ArmConstants.kTimeoutMs);

    arm_motor.configMotionCruiseVelocity(3000, PidConstants.ArmConstants.kTimeoutMs);
    arm_motor.configMotionAcceleration(4000, PidConstants.ArmConstants.kTimeoutMs);

    arm_motor.setSelectedSensorPosition(0, PidConstants.ArmConstants.kPIDLoopIdx,
        PidConstants.ArmConstants.kTimeoutMs);

    arm_motor.setNeutralMode(NeutralMode.Brake);

    arm_motor.configMotionSCurveStrength(25);

  }

  @Override
  public void periodic() {

  }

  public void reset_encoder() {
    arm_motor.setSelectedSensorPosition(0);
  }

  public static void arm_motor_set(double hiz) {
    arm_motor.set(hiz);
  }

  // Units to cm
  public static int arm_uzunluk_cm() {
    return (int) (Math.round(arm_motor.getSelectedSensorPosition() / 2048 / ArmConstants.arm_gear_ratio
        * ArmConstants.arm_distance_per_rotation));
  }

  // cm to Units
  public static int arm_uzunluk_units(int cm) {
    return (int) (2048 * ArmConstants.arm_gear_ratio * cm / ArmConstants.arm_distance_per_rotation);
  }
}
