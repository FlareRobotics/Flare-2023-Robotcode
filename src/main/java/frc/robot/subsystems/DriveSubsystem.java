package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;

import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.TalonFXInvertType;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.sensors.Pigeon2;

import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.DriveConstants;

public class DriveSubsystem extends SubsystemBase {
  public static WPI_TalonFX leftFrontMotor = new WPI_TalonFX(Constants.DriveConstants.solon_falcon_port);
  public static WPI_TalonFX leftRearMotor = new WPI_TalonFX(Constants.DriveConstants.solarka_falcon_port);

  public static WPI_TalonFX rightFrontMotor = new WPI_TalonFX(Constants.DriveConstants.sagon_falcon_port);
  public static WPI_TalonFX rightRearMotor = new WPI_TalonFX(Constants.DriveConstants.sagarka_falcon_port);

  public static DifferentialDrive drive_ = new DifferentialDrive(leftFrontMotor, rightFrontMotor);

  public static Pigeon2 m_gyro = new Pigeon2(Constants.DriveConstants.pigeon_port);

  /** Config Objects for motor controllers */
  TalonFXConfiguration _rightConfig = new TalonFXConfiguration();


  public DriveSubsystem() {
    leftFrontMotor.configFactoryDefault();
    leftRearMotor.configFactoryDefault();
    rightFrontMotor.configFactoryDefault();
    rightRearMotor.configFactoryDefault();


    leftFrontMotor.setInverted(TalonFXInvertType.Clockwise);
    leftRearMotor.setInverted(TalonFXInvertType.Clockwise);
    rightFrontMotor.setInverted(TalonFXInvertType.CounterClockwise);
    rightRearMotor.setInverted(TalonFXInvertType.CounterClockwise);

    /* Disable all motors */
    rightRearMotor.set(TalonFXControlMode.PercentOutput, 0);
    leftRearMotor.set(TalonFXControlMode.PercentOutput, 0);
    rightFrontMotor.set(TalonFXControlMode.PercentOutput, 0);
    leftFrontMotor.set(TalonFXControlMode.PercentOutput, 0);
    leftFrontMotor.setNeutralMode(NeutralMode.Coast);
    leftRearMotor.setNeutralMode(NeutralMode.Coast);

    rightFrontMotor.setNeutralMode(NeutralMode.Coast);
    rightRearMotor.setNeutralMode(NeutralMode.Coast);
    /* Set neutral modes */
   

    // rightFrontMotor.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor,
    //     PidConstants.DriveConstants.kPIDLoopIdx,
    //     PidConstants.DriveConstants.kTimeoutMs);
    // rightFrontMotor.configNeutralDeadband(0.001, PidConstants.DriveConstants.kTimeoutMs);
    // rightFrontMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_13_Base_PIDF0, 10,
    //     PidConstants.DriveConstants.kTimeoutMs);
    // rightFrontMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_10_MotionMagic, 10,
    //     PidConstants.DriveConstants.kTimeoutMs);
    // rightFrontMotor.configNominalOutputForward(0, PidConstants.DriveConstants.kTimeoutMs);
    // rightFrontMotor.configNominalOutputReverse(0, PidConstants.DriveConstants.kTimeoutMs);
    // rightFrontMotor.configPeakOutputForward(PidConstants.DriveConstants.kGains.kPeakOutput,
    //     PidConstants.DriveConstants.kTimeoutMs);
    // rightFrontMotor.configPeakOutputReverse(-PidConstants.DriveConstants.kGains.kPeakOutput,
    //     PidConstants.DriveConstants.kTimeoutMs);
    // rightFrontMotor.selectProfileSlot(PidConstants.DriveConstants.kSlotIdx,
    //     PidConstants.DriveConstants.kPIDLoopIdx);
    // rightFrontMotor.config_kF(PidConstants.DriveConstants.kSlotIdx, PidConstants.DriveConstants.kGains.kF,
    //     PidConstants.DriveConstants.kTimeoutMs);
    // rightFrontMotor.config_kP(PidConstants.DriveConstants.kSlotIdx, PidConstants.DriveConstants.kGains.kP,
    //     PidConstants.DriveConstants.kTimeoutMs);
    // rightFrontMotor.config_kI(PidConstants.DriveConstants.kSlotIdx, PidConstants.DriveConstants.kGains.kI,
    //     PidConstants.DriveConstants.kTimeoutMs);
    // rightFrontMotor.config_kD(PidConstants.DriveConstants.kSlotIdx, PidConstants.DriveConstants.kGains.kD,
    //     PidConstants.DriveConstants.kTimeoutMs);

    // /* Set acceleration and vcruise velocity - see documentation */
    // rightFrontMotor.configMotionCruiseVelocity(15000, PidConstants.DriveConstants.kTimeoutMs);
    // rightFrontMotor.configMotionAcceleration(18000, PidConstants.DriveConstants.kTimeoutMs);

    // /* Zero the sensor once on robot boot up */
    // rightFrontMotor.setSelectedSensorPosition(0, PidConstants.DriveConstants.kPIDLoopIdx,
    //     PidConstants.DriveConstants.kTimeoutMs);

    // rightFrontMotor.configMotionSCurveStrength(1);

    // leftFrontMotor.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor,
    //     PidConstants.DriveConstants.kPIDLoopIdx,
    //     PidConstants.DriveConstants.kTimeoutMs);
    // leftFrontMotor.configNeutralDeadband(0.001, PidConstants.DriveConstants.kTimeoutMs);
    // leftFrontMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_13_Base_PIDF0, 10,
    //     PidConstants.DriveConstants.kTimeoutMs);
    // leftFrontMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_10_MotionMagic, 10,
    //     PidConstants.DriveConstants.kTimeoutMs);
    // leftFrontMotor.configNominalOutputForward(0, PidConstants.DriveConstants.kTimeoutMs);
    // leftFrontMotor.configNominalOutputReverse(0, PidConstants.DriveConstants.kTimeoutMs);
    // leftFrontMotor.configPeakOutputForward(PidConstants.DriveConstants.kGains.kPeakOutput,
    //     PidConstants.DriveConstants.kTimeoutMs);
    // leftFrontMotor.configPeakOutputReverse(-PidConstants.DriveConstants.kGains.kPeakOutput,
    //     PidConstants.DriveConstants.kTimeoutMs);
    // leftFrontMotor.selectProfileSlot(PidConstants.DriveConstants.kSlotIdx,
    //     PidConstants.DriveConstants.kPIDLoopIdx);
    // leftFrontMotor.config_kF(PidConstants.DriveConstants.kSlotIdx, PidConstants.DriveConstants.kGains.kF,
    //     PidConstants.DriveConstants.kTimeoutMs);
    // leftFrontMotor.config_kP(PidConstants.DriveConstants.kSlotIdx, PidConstants.DriveConstants.kGains.kP,
    //     PidConstants.DriveConstants.kTimeoutMs);
    // leftFrontMotor.config_kI(PidConstants.DriveConstants.kSlotIdx, PidConstants.DriveConstants.kGains.kI,
    //     PidConstants.DriveConstants.kTimeoutMs);
    // leftFrontMotor.config_kD(PidConstants.DriveConstants.kSlotIdx, PidConstants.DriveConstants.kGains.kD,
    //     PidConstants.DriveConstants.kTimeoutMs);

    // /* Set acceleration and vcruise velocity - see documentation */
    // leftFrontMotor.configMotionCruiseVelocity(15000, PidConstants.DriveConstants.kTimeoutMs);
    // leftFrontMotor.configMotionAcceleration(18000, PidConstants.DriveConstants.kTimeoutMs);

    // /* Zero the sensor once on robot boot up */
    // leftFrontMotor.setSelectedSensorPosition(0, PidConstants.DriveConstants.kPIDLoopIdx,
    //     PidConstants.DriveConstants.kTimeoutMs);

    // leftFrontMotor.configMotionSCurveStrength(1);

    leftRearMotor.follow(leftFrontMotor);
    rightRearMotor.follow(rightFrontMotor);

    zeroSensors();
    setDash();

 
  }

  void setDash() {

    SmartDashboard.putNumber("Pigeon Yaw", m_gyro.getYaw());
    SmartDashboard.putNumber("Pigeon Pitch", m_gyro.getPitch());
    SmartDashboard.putNumber("Pigeon Heading", getHeading());
    SmartDashboard.putNumber("Meters Encoder", getAverageEncoderDistance());
    SmartDashboard.putBoolean("Compressor Switch", ClawSubsystem.Compressor.getPressureSwitchValue());
    SmartDashboard.putBoolean("Compressor Enabled", ClawSubsystem.Compressor.isEnabled());
  }



  public static void zeroSensors() {
    rightFrontMotor.getSensorCollection().setIntegratedSensorPosition(0, Constants.kTimeoutMs);
    leftFrontMotor.getSensorCollection().setIntegratedSensorPosition(0, Constants.kTimeoutMs);
    m_gyro.setYaw(0);
  }

  @Override
  public void periodic() {
    setDash();
  }

  public double get_encoder_distance() {
    return ((leftFrontMotor.getSelectedSensorPosition(0) +
        leftRearMotor.getSelectedSensorPosition(0) +
        rightFrontMotor.getSelectedSensorPosition(0) +
        rightRearMotor.getSelectedSensorPosition(0)) / 4);
  }

  public void reset_encoders() {
    leftFrontMotor.setSelectedSensorPosition(0);
    leftRearMotor.setSelectedSensorPosition(0);
    rightFrontMotor.setSelectedSensorPosition(0);
    rightRearMotor.setSelectedSensorPosition(0);
  }

  public static double getRightEncoderDistance() {
    return rightFrontMotor.getSelectedSensorPosition() * 47.85d / DriveConstants.drive_disli_orani / 2048;
  }

  public static double getLeftEncoderDistance() {
    return leftFrontMotor.getSelectedSensorPosition() * 47.85d / DriveConstants.drive_disli_orani / 2048;
  }

  public static double getAverageEncoderDistance() {
    return (getRightEncoderDistance() + getLeftEncoderDistance()) / (2.0) / 1.8d;
  }

  public void tankDriveVolts(double leftVolts, double rightVolts) {
    leftRearMotor.setVoltage(leftVolts);
    leftFrontMotor.setVoltage(leftVolts);
    /*
     * if (rightVolts >= 0) {
     * rightVolts = rightVolts + 1;
     * } else {
     * rightVolts = rightVolts - 1;
     * }
     */
    rightRearMotor.setVoltage(-rightVolts);
    rightFrontMotor.setVoltage(-rightVolts);
  }

  public void setMaxOutput(double maxOutput) {
  }

  public DifferentialDriveWheelSpeeds getWheelSpeeds() {
    return new DifferentialDriveWheelSpeeds(
        10.0 *
            rightRearMotor.getSelectedSensorVelocity() *
            (1.0 / DriveConstants.kEncoderCPR) *
            (Math.PI * DriveConstants.kWheelDiameterMeters),
        10.0 *
            leftRearMotor.getSelectedSensorVelocity() *
            (1.0 / DriveConstants.kEncoderCPR) *
            (-Math.PI * DriveConstants.kWheelDiameterMeters));
  }

  public static void arcadeDrive(double fwd, double rot) {
    fwd = Deadband(fwd);
    rot = Deadband(rot);
    
    drive_.arcadeDrive(fwd, rot);
  }

  public static void pidDrive(double forward) {
    forward = Deadband(forward);
    double target_sensorUnits = (forward / 47.85d) * 2048 * DriveConstants.drive_disli_orani;

    rightRearMotor.set(TalonFXControlMode.MotionMagic, target_sensorUnits);
    leftRearMotor.set(TalonFXControlMode.Follower, 3);
  }

  public static void turnDegrees(double degree) {
    arcadeDrive(0, degree < 0 ? 0.4d : -0.4d);
  }

  static double Deadband(double value) {
    /* Upper deadband */
    if (value >= +0.15)
      return value;

    /* Lower deadband */
    if (value <= -0.15)
      return value;

    /* Outside deadband */
    return 0;
  }

  static double turnDeadband(double value) {
    /* Upper deadband */
    if (value >= +0.05)
      return value;

    /* Lower deadband */
    if (value <= -0.05)
      return value;

    /* Outside deadband */
    return 0;
  }


  public void zeroHeading() {
    m_gyro.setYaw(0);

  }

  public double getHeading() {
    return (Math.IEEEremainder(m_gyro.getYaw(), 360) * -1);
  }

  public void resetEncoders() {
    rightFrontMotor.setSelectedSensorPosition(0);
    rightRearMotor.setSelectedSensorPosition(0);

    leftFrontMotor.setSelectedSensorPosition(0);
    leftRearMotor.setSelectedSensorPosition(0);
  }

  public static boolean turn_angles(double angle, double startangle, Boolean sag) {
    if (sag) {
      if (m_gyro.getYaw() < startangle + angle) {
        arcadeDrive(0, 0.5);
      } else {
        return true;
      }
    } else {
      if (m_gyro.getYaw() > startangle + angle) {
        arcadeDrive(0, -0.5);
      } else {
        return true;
      }
    }
    return false;
  }

  public static boolean drive_PID_centimeters(double cm) {
    var target_sensorUnits = cm / (Units.inchesToMeters(6) * Math.PI * 2) * DriveConstants.drive_disli_orani * 2048;
    rightFrontMotor.set(TalonFXControlMode.MotionMagic, target_sensorUnits);
    leftFrontMotor.set(TalonFXControlMode.MotionMagic, target_sensorUnits);
    return Math.abs(getAverageEncoderDistance()) > Math.abs(cm);
  }
}