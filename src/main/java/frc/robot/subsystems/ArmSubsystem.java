
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

    /* Factory default hardware to prevent unexpected behavior */
    arm_motor.configFactoryDefault();

    /* Configure Sensor Source for Pirmary PID */
    arm_motor.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor,
        PidConstants.TurretConstants.kPIDLoopIdx,
        PidConstants.TurretConstants.kTimeoutMs);

    /*
     * set deadband to super small 0.001 (0.1 %).
     * The default deadband is 0.04 (4 %)
     */
    arm_motor.configNeutralDeadband(0.001, PidConstants.TurretConstants.kTimeoutMs);

    /**
     * Configure Talon FX Output and Sensor direction accordingly Invert Motor to
     * have green LEDs when driving Talon Forward / Requesting Postiive Output Phase
     * sensor to have positive increment when driving Talon Forward (Green LED)
     */
    arm_motor.setSensorPhase(false);
    arm_motor.setInverted(false);
    /*
     * Talon FX does not need sensor phase set for its integrated sensor
     * This is because it will always be correct if the selected feedback device is
     * integrated sensor (default value)
     * and the user calls getSelectedSensor* to get the sensor's position/velocity.
     * 
     * https://phoenix-documentation.readthedocs.io/en/latest/ch14_MCSensor.html#
     * sensor-phase
     */
    // arm_motor.setSensorPhase(true);

    /* Set relevant frame periods to be at least as fast as periodic rate */
    arm_motor.setStatusFramePeriod(StatusFrameEnhanced.Status_13_Base_PIDF0, 10,
        PidConstants.TurretConstants.kTimeoutMs);
    arm_motor.setStatusFramePeriod(StatusFrameEnhanced.Status_10_MotionMagic, 10,
        PidConstants.TurretConstants.kTimeoutMs);

    /* Set the peak and nominal outputs */
    arm_motor.configNominalOutputForward(0, PidConstants.TurretConstants.kTimeoutMs);
    arm_motor.configNominalOutputReverse(0, PidConstants.TurretConstants.kTimeoutMs);
    arm_motor.configPeakOutputForward(PidConstants.TurretConstants.kGains.kPeakOutput,
        PidConstants.TurretConstants.kTimeoutMs);
    arm_motor.configPeakOutputReverse(-PidConstants.TurretConstants.kGains.kPeakOutput,
        PidConstants.TurretConstants.kTimeoutMs);

    /* Set Motion Magic gains in slot0 - see documentation */
    arm_motor.selectProfileSlot(PidConstants.TurretConstants.kSlotIdx,
        PidConstants.TurretConstants.kPIDLoopIdx);
    arm_motor.config_kF(PidConstants.TurretConstants.kSlotIdx, PidConstants.TurretConstants.kGains.kF,
        PidConstants.TurretConstants.kTimeoutMs);
    arm_motor.config_kP(PidConstants.TurretConstants.kSlotIdx, PidConstants.TurretConstants.kGains.kP,
        PidConstants.TurretConstants.kTimeoutMs);
    arm_motor.config_kI(PidConstants.TurretConstants.kSlotIdx, PidConstants.TurretConstants.kGains.kI,
        PidConstants.TurretConstants.kTimeoutMs);
    arm_motor.config_kD(PidConstants.TurretConstants.kSlotIdx, PidConstants.TurretConstants.kGains.kD,
        PidConstants.TurretConstants.kTimeoutMs);

    /* Set acceleration and vcruise velocity - see documentation */
    arm_motor.configMotionCruiseVelocity(3000, PidConstants.TurretConstants.kTimeoutMs);
    arm_motor.configMotionAcceleration(4000, PidConstants.TurretConstants.kTimeoutMs);

    /* Zero the sensor once on robot boot up */
    arm_motor.setSelectedSensorPosition(0, PidConstants.TurretConstants.kPIDLoopIdx,
        PidConstants.TurretConstants.kTimeoutMs);

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
