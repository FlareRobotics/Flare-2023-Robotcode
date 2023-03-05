package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.ArmConstants;
import frc.robot.PID.PidConstants;

public class ArmSubsystem extends SubsystemBase {
  public static WPI_TalonFX arm_motor = new WPI_TalonFX(ArmConstants.arm_motor_port);

  public ArmSubsystem() {

    arm_motor.configFactoryDefault();

    arm_motor.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor,
            Constants.kPIDLoopIdx,
            Constants.kTimeoutMs);

    arm_motor.configNeutralDeadband(0.001, Constants.kTimeoutMs);

    arm_motor.setSensorPhase(false);
    arm_motor.setInverted(false);

    arm_motor.setStatusFramePeriod(StatusFrameEnhanced.Status_13_Base_PIDF0, 10,
            Constants.kTimeoutMs);
    arm_motor.setStatusFramePeriod(StatusFrameEnhanced.Status_10_MotionMagic, 10,
            Constants.kTimeoutMs);

    arm_motor.configNominalOutputForward(0, Constants.kTimeoutMs);
    arm_motor.configNominalOutputReverse(0, Constants.kTimeoutMs);
    arm_motor.configPeakOutputForward(PidConstants.ArmConstants.kGains.kPeakOutput,
            Constants.kTimeoutMs);
    arm_motor.configPeakOutputReverse(-PidConstants.ArmConstants.kGains.kPeakOutput,
            Constants.kTimeoutMs);

    arm_motor.selectProfileSlot(Constants.kSlotIdx,
            Constants.kPIDLoopIdx);
    arm_motor.config_kF(Constants.kSlotIdx, PidConstants.ArmConstants.kGains.kF,
            Constants.kTimeoutMs);
    arm_motor.config_kP(Constants.kSlotIdx, PidConstants.ArmConstants.kGains.kP,
            Constants.kTimeoutMs);
    arm_motor.config_kI(Constants.kSlotIdx, PidConstants.ArmConstants.kGains.kI,
            Constants.kTimeoutMs);
    arm_motor.config_kD(Constants.kSlotIdx, PidConstants.ArmConstants.kGains.kD,
            Constants.kTimeoutMs);

    arm_motor.configMotionCruiseVelocity(3000, Constants.kTimeoutMs);
    arm_motor.configMotionAcceleration(4000, Constants.kTimeoutMs);

    arm_motor.setSelectedSensorPosition(0, Constants.kPIDLoopIdx,
            Constants.kTimeoutMs);

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
