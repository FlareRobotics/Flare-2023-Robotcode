package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.RemoteSensorSource;
import com.ctre.phoenix.motorcontrol.StatusFrame;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.TalonFXInvertType;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.sensors.Pigeon2;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.DriveConstants;
import frc.robot.PID.PidConstants;

public class DriveSubsystem extends SubsystemBase {
  public static WPI_TalonFX leftFrontMotor = new WPI_TalonFX(Constants.DriveConstants.solon_falcon_port);
  public static WPI_TalonFX leftRearMotor = new WPI_TalonFX(Constants.DriveConstants.solarka_falcon_port);

  public static WPI_TalonFX rightFrontMotor = new WPI_TalonFX(Constants.DriveConstants.sagon_falcon_port);
  public static WPI_TalonFX rightRearMotor = new WPI_TalonFX(Constants.DriveConstants.sagarka_falcon_port);

  public static DifferentialDrive m_drive = new DifferentialDrive(leftRearMotor, rightRearMotor);

  public static Pigeon2 m_gyro = new Pigeon2(0);

  TalonFXConfiguration _leftConfig = new TalonFXConfiguration();
  TalonFXConfiguration _rightConfig = new TalonFXConfiguration();

  private NeutralMode defaultMode = NeutralMode.Coast;
  private final DifferentialDriveOdometry m_odometry;

  private final Field2d m_field = new Field2d();

  public DriveSubsystem() {

    // Pose2D(x,y) is the distance from the bottom left of the field!!
    m_odometry = new DifferentialDriveOdometry(Rotation2d.fromDegrees(getHeading()), getLeftEncoderDistance(),
        getRightEncoderDistance(), new Pose2d(1.0, 1.0, new Rotation2d()));
    SmartDashboard.putData("Field", m_field);

    leftFrontMotor.setInverted(TalonFXInvertType.Clockwise);
    leftRearMotor.setInverted(TalonFXInvertType.Clockwise);
    rightFrontMotor.setInverted(TalonFXInvertType.CounterClockwise);
    rightRearMotor.setInverted(TalonFXInvertType.CounterClockwise);

    /* Disable all motors */
    rightRearMotor.set(TalonFXControlMode.PercentOutput, 0);
    leftRearMotor.set(TalonFXControlMode.PercentOutput, 0);
    rightFrontMotor.set(TalonFXControlMode.PercentOutput, 0);
    leftFrontMotor.set(TalonFXControlMode.PercentOutput, 0);

    /* Set neutral modes */
    leftRearMotor.setNeutralMode(defaultMode);
    rightRearMotor.setNeutralMode(defaultMode);
    leftFrontMotor.setNeutralMode(defaultMode);
    rightFrontMotor.setNeutralMode(defaultMode);

    leftFrontMotor.follow(leftRearMotor);
    rightFrontMotor.follow(rightRearMotor);
    _leftConfig.primaryPID.selectedFeedbackSensor = TalonFXFeedbackDevice.IntegratedSensor.toFeedbackDevice();

    _rightConfig.remoteFilter0.remoteSensorDeviceID = leftRearMotor.getDeviceID(); // Device ID of Remote Source
    _rightConfig.remoteFilter0.remoteSensorSource = RemoteSensorSource.TalonFX_SelectedSensor; // Remote Source Type

    setRobotDistanceConfigs(TalonFXInvertType.Clockwise, _rightConfig);

    _rightConfig.slot0.kF = PidConstants.DriveConstants.kGains_Distanc.kF;
    _rightConfig.slot0.kP = PidConstants.DriveConstants.kGains_Distanc.kP;
    _rightConfig.slot0.kI = PidConstants.DriveConstants.kGains_Distanc.kI;
    _rightConfig.slot0.kD = PidConstants.DriveConstants.kGains_Distanc.kD;
    _rightConfig.slot0.integralZone = PidConstants.DriveConstants.kGains_Distanc.kIzone;
    _rightConfig.slot0.closedLoopPeakOutput = PidConstants.DriveConstants.kGains_Distanc.kPeakOutput;

    _rightConfig.auxPIDPolarity = false;

    _rightConfig.slot1.kF = PidConstants.DriveConstants.kGains_Turning.kF;
    _rightConfig.slot1.kP = PidConstants.DriveConstants.kGains_Turning.kP;
    _rightConfig.slot1.kI = PidConstants.DriveConstants.kGains_Turning.kI;
    _rightConfig.slot1.kD = PidConstants.DriveConstants.kGains_Turning.kD;
    _rightConfig.slot1.integralZone = PidConstants.DriveConstants.kGains_Turning.kIzone;
    _rightConfig.slot1.closedLoopPeakOutput = PidConstants.DriveConstants.kGains_Turning.kPeakOutput;

    _leftConfig.neutralDeadband = PidConstants.DriveConstants.kNeutralDeadband;
    _rightConfig.neutralDeadband = PidConstants.DriveConstants.kNeutralDeadband;

    int closedLoopTimeMs = 1;
    _rightConfig.slot0.closedLoopPeriod = closedLoopTimeMs;
    _rightConfig.slot1.closedLoopPeriod = closedLoopTimeMs;
    _rightConfig.slot2.closedLoopPeriod = closedLoopTimeMs;
    _rightConfig.slot3.closedLoopPeriod = closedLoopTimeMs;

    /* Motion Magic Configs */
    _rightConfig.motionAcceleration = 2000; // (distance units per 100 ms) per second
    _rightConfig.motionCruiseVelocity = 2000; // distance units per 100 ms

    leftRearMotor.configAllSettings(_leftConfig);
    rightRearMotor.configAllSettings(_rightConfig);

    rightRearMotor.setStatusFramePeriod(StatusFrame.Status_12_Feedback1, 20, Constants.kTimeoutMs);
    rightRearMotor.setStatusFramePeriod(StatusFrame.Status_13_Base_PIDF0, 20, Constants.kTimeoutMs);
    leftRearMotor.setStatusFramePeriod(StatusFrame.Status_2_Feedback0, 5, Constants.kTimeoutMs);

    rightRearMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_10_Targets, 10);
    rightRearMotor.configMotionSCurveStrength(0);
    rightRearMotor.selectProfileSlot(PidConstants.DriveConstants.kSlot_Distanc,
        PidConstants.DriveConstants.PID_PRIMARY);
    zeroSensors();
  }

  void setDash() {
    // SmartDashboard.putNumber("Encoder Distance M", getRightEncoderDistance());
  }

  void zeroSensors() {
    leftRearMotor.getSensorCollection().setIntegratedSensorPosition(0, Constants.kTimeoutMs);
    rightRearMotor.getSensorCollection().setIntegratedSensorPosition(0, Constants.kTimeoutMs);
    m_gyro.setYaw(0);
    System.out.println("[Integrated Sensors] All sensors are zeroed.\n");
  }

  @Override
  public void periodic() {
    m_odometry.update(
        Rotation2d.fromDegrees(getHeading()),
        getLeftEncoderDistance(),
        getRightEncoderDistance());

    // m_field.setRobotPose(
    // m_odometry.getPoseMeters().getX(),
    // -m_odometry.getPoseMeters().getY(),
    // new Rotation2d(Math.toRadians(-getHeading())));
    m_field.setRobotPose(m_odometry.getPoseMeters());

    // Ayrıca field_2D'de trajectory görmek için
    // _field.getObject("traj").setTrajectory(m_trajectory);
    // kullanabiliriz!!
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
    return rightRearMotor.getSelectedSensorPosition()
        * (Constants.DriveConstants.drive_disli_orani / Constants.DriveConstants.kEncoderCPR)
        * (Math.PI * Constants.DriveConstants.kWheelDiameterMeters);
  }

  public static double getLeftEncoderDistance() {
    return leftRearMotor.getSelectedSensorPosition()
        * (Constants.DriveConstants.drive_disli_orani / Constants.DriveConstants.kEncoderCPR)
        * (-Math.PI * Constants.DriveConstants.kWheelDiameterMeters);
  }

  public static double getAverageEncoderDistance() {
    return (getRightEncoderDistance() + getLeftEncoderDistance()) / (2.0);
  }

  public void tankDriveVolts(double leftVolts, double rightVolts) {
    leftRearMotor.setVoltage(leftVolts);
    rightRearMotor.setVoltage(rightVolts);
    m_drive.feed();
  }

  public void setMaxOutput(double maxOutput) {
    m_drive.setMaxOutput(maxOutput);
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

  public void resetOdometry(Pose2d pose) {
    resetEncoders();
    m_odometry.resetPosition(Rotation2d.fromDegrees(getHeading()), getLeftEncoderDistance(), getRightEncoderDistance(),
        new Pose2d(5.0, 5.0, new Rotation2d()));
  }

  public static void arcadeDrive(double fwd, double rot) {
    fwd = Deadband(fwd);
    rot = Deadband(rot);

    m_drive.arcadeDrive(fwd, rot);
  }

  public static void pidDrive(double forward, double turn) {
    forward = Deadband(forward);
    turn = Deadband(turn);

    double target_sensorUnits = forward * PidConstants.DriveConstants.kSensorUnitsPerRotation
        * PidConstants.DriveConstants.kRotationsToTravel;

    rightRearMotor.set(TalonFXControlMode.MotionMagic, target_sensorUnits);
    leftRearMotor.set(TalonFXControlMode.MotionMagic, target_sensorUnits);
  }

  static double Deadband(double value) {
    if (value >= +0.25)
      return value;

    if (value <= -0.25)
      return value;

    return 0;
  }

  void setRobotDistanceConfigs(TalonFXInvertType masterInvertType, TalonFXConfiguration masterConfig) {
    if (masterInvertType == TalonFXInvertType.Clockwise) {

      masterConfig.diff0Term = TalonFXFeedbackDevice.IntegratedSensor.toFeedbackDevice();
      masterConfig.diff1Term = TalonFXFeedbackDevice.RemoteSensor0.toFeedbackDevice();
      masterConfig.primaryPID.selectedFeedbackSensor = TalonFXFeedbackDevice.SensorDifference.toFeedbackDevice();
    } else {
      masterConfig.sum0Term = TalonFXFeedbackDevice.RemoteSensor0.toFeedbackDevice();
      masterConfig.sum1Term = TalonFXFeedbackDevice.IntegratedSensor.toFeedbackDevice();
      masterConfig.primaryPID.selectedFeedbackSensor = TalonFXFeedbackDevice.SensorSum.toFeedbackDevice();
    }

    masterConfig.primaryPID.selectedFeedbackCoefficient = 0.5;
  }

  public void zeroHeading() {
    m_gyro.setYaw(0);

  }

  public Pose2d getPose() {
    return m_odometry.getPoseMeters();
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
        pidDrive(0, 0.5);
      } else {
        return true;
      }
    } else {
      if (m_gyro.getYaw() > startangle + angle) {
        pidDrive(0, -0.5);
      } else {
        return true;
      }
    }
    return false;
  }

  public static boolean drive_PID_centimeters(double cm) {
    var target_sensorUnits = cm / (Units.inchesToMeters(6) * Math.PI * 2) * DriveConstants.drive_disli_orani * 2048;
    rightRearMotor.set(TalonFXControlMode.MotionMagic, target_sensorUnits);
    leftRearMotor.set(TalonFXControlMode.MotionMagic, target_sensorUnits);
    return (getAverageEncoderDistance() >= cm);
  }
}