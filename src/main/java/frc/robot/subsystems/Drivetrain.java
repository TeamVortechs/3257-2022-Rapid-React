package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.IMotorController;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.wpilibj.SpeedController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.utils.Conversions;

public class Drivetrain extends SubsystemBase {
  private WPI_TalonFX backLeft = new WPI_TalonFX(Constants.DriveConstants.backLeftPort);
  
  private WPI_TalonFX backRight = new WPI_TalonFX(Constants.DriveConstants.backRightPort);
  
  private WPI_TalonFX frontLeft = new WPI_TalonFX(Constants.DriveConstants.frontLeftPort);
  
  private WPI_TalonFX frontRight = new WPI_TalonFX(Constants.DriveConstants.frontRightPort);
  
  private DifferentialDrive differentialDrive = new DifferentialDrive((SpeedController)this.frontLeft, (SpeedController)this.frontRight);
  
  private DifferentialDriveKinematics kinematics = new DifferentialDriveKinematics(Constants.DriveConstants.trackwidth);
  
  private DifferentialDriveOdometry odometry = new DifferentialDriveOdometry(new Rotation2d(), new Pose2d());
  
  private SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(0.7D, 1.98D, 0.2D);
  
  private AHRS gyro = new AHRS();
  
  private Field2d field = new Field2d();
  
  public Drivetrain() {
    this.gyro.calibrate();
    this.gyro.reset();
    this.backLeft.configFactoryDefault();
    this.frontLeft.configFactoryDefault();
    this.backRight.configFactoryDefault();
    this.frontRight.configFactoryDefault();
    this.frontLeft.setInverted(InvertType.None);
    this.frontLeft.setNeutralMode(NeutralMode.Coast);
    this.frontLeft.configOpenloopRamp(0.0D);
    this.frontLeft.setSensorPhase(true);
    this.frontLeft.config_kP(0, Constants.DriveConstants.talonFXP, 30);
    this.frontLeft.config_kI(0, Constants.DriveConstants.talonFXI, 30);
    this.frontLeft.config_kD(0, Constants.DriveConstants.talonFXD, 30);
    this.frontLeft.config_kF(0, Constants.DriveConstants.talonFXF, 30);
    this.backLeft.follow((IMotorController)this.frontLeft);
    this.backLeft.setNeutralMode(NeutralMode.Coast);
    this.backLeft.setInverted(InvertType.FollowMaster);
    this.frontRight.setInverted(InvertType.None);
    this.frontRight.setNeutralMode(NeutralMode.Coast);
    this.frontRight.configOpenloopRamp(0.0D);
    this.frontRight.config_kP(0, Constants.DriveConstants.talonFXP, 30);
    this.frontRight.config_kI(0, Constants.DriveConstants.talonFXI, 30);
    this.frontRight.config_kD(0, Constants.DriveConstants.talonFXD, 30);
    this.frontRight.config_kF(0, Constants.DriveConstants.talonFXF, 30);
    this.backRight.follow((IMotorController)this.frontRight);
    this.backRight.setInverted(InvertType.FollowMaster);
    this.backRight.setNeutralMode(NeutralMode.Coast);
    SmartDashboard.putData("Field", (Sendable)this.field);
  }
  
  public void periodic() {
    this.odometry.update(Rotation2d.fromDegrees(getHeading()), getLeftEncoderPosition(), getRightEncoderPosition());
    this.field.setRobotPose(getPose());
    SmartDashboard.putData("field", (Sendable)this.field);
    SmartDashboard.putNumber("right pos", getRightEncoderPosition());
    SmartDashboard.putNumber("left pos", getLeftEncoderPosition());
    SmartDashboard.putNumber("gyro", getHeading());
  }
  
  public void tankDriveVolts(double left, double right) {
    tankDrive(left / 12.0D, right / 12.0D);
  }
  
  public void tankDrive(double left, double right) {
    this.differentialDrive.tankDrive(left, right);
  }
  
  public void setWheelRPM(double leftRPM, double rightRPM) {
    this.frontLeft.set(TalonFXControlMode.Velocity, Conversions.TalonFXConversions.RPM2Native(leftRPM));
    this.frontRight.set(TalonFXControlMode.Velocity, Conversions.TalonFXConversions.RPM2Native(rightRPM));
    this.differentialDrive.feed();
  }
  
  public void setWheelSpeeds(DifferentialDriveWheelSpeeds speeds) {
    this.frontLeft.set(TalonFXControlMode.Velocity, metersPerSecondToNative(speeds.leftMetersPerSecond));
    this.frontRight.set(TalonFXControlMode.Velocity, metersPerSecondToNative(speeds.rightMetersPerSecond));
  }
  
  public void arcadeDrive(double throttle, double turn) {
    System.out.println("throttle " + Math.round(throttle * 10.0D) / 10.0D + " turn " + Math.round(turn * 10.0D) / 10.0D + " pose " + getPose());
    this.differentialDrive.arcadeDrive(throttle, turn);
  }
  
  public void setNeutralMode(NeutralMode mode) {
    this.frontLeft.setNeutralMode(mode);
    this.frontRight.setNeutralMode(mode);
    this.backLeft.setNeutralMode(mode);
    this.backRight.setNeutralMode(mode);
  }
  
  public void setSlowMode(boolean on) {
    this.differentialDrive.setMaxOutput(on ? 0.2D : 1.0D);
  }
  
  public void resetOdometry(Pose2d pose) {
    this.odometry.resetPosition(pose, pose.getRotation());
    this.frontLeft.setSelectedSensorPosition(0.0D);
    this.frontRight.setSelectedSensorPosition(0.0D);
    this.gyro.reset();
  }
  
  public void resetOdometry() {
    resetOdometry(new Pose2d());
  }
  
  public static double nativePositionToMeters(double nativeUnits) {
    double wheelRotations = Conversions.TalonFXConversions.Native2Rotations(nativeUnits) * Constants.DriveConstants.gearboxRatio;
    return wheelRotations * Math.PI * Constants.DriveConstants.wheelDiameter;
  }
  
  public static double metersToNativePosition(double meters) {
    double wheelRotations = meters / Math.PI * Constants.DriveConstants.wheelDiameter;
    double motorRotations = wheelRotations / Constants.DriveConstants.gearboxRatio;
    return Conversions.TalonFXConversions.Rotations2Native(motorRotations);
  }
  
  public static double metersPerSecondToNative(double metersPerSecond) {
    double wheelRPS = metersPerSecond / Math.PI * Constants.DriveConstants.wheelDiameter;
    double motorRPS = wheelRPS * Constants.DriveConstants.gearboxRatio;
    return Conversions.TalonFXConversions.RPM2Native(motorRPS / 60.0D);
  }
  
  public static double nativeToMetersPerSecond(double nativeUnits) {
    double motorRPS = Conversions.TalonFXConversions.Native2RPM(nativeUnits) * 60.0D / Constants.DriveConstants.gearboxRatio;
    return motorRPS * Math.PI * Constants.DriveConstants.wheelDiameter;
  }
  
  public PIDController getLeftController() {
    return null;
  }
  
  public PIDController getRightController() {
    return null;
  }
  
  public SimpleMotorFeedforward getFeedForward() {
    return this.feedforward;
  }
  
  public double getLeftEncoderPosition() {
    return nativePositionToMeters((this.frontLeft.getSelectedSensorPosition() + this.backLeft.getSelectedSensorPosition()) / 2.0D);
  }
  
  public double getLeftEncoderVelocity() {
    return nativeToMetersPerSecond((this.frontLeft.getSelectedSensorVelocity() + this.backLeft.getSelectedSensorVelocity()) / 2.0D * 10.0D);
  }
  
  public double getRightEncoderPosition() {
    return nativePositionToMeters((this.frontRight.getSelectedSensorPosition() + this.backRight.getSelectedSensorPosition()) / 2.0D);
  }
  
  public double getRightEncoderVelocity() {
    return nativeToMetersPerSecond((this.frontRight.getSelectedSensorVelocity() + this.backRight.getSelectedSensorVelocity()) / 2.0D * 10.0D);
  }
  
  public double getHeading() {
    return Constants.DriveConstants.invertGyro ? -this.gyro.getAngle() : this.gyro.getAngle();
  }
  
  public double getHeadingRate() {
    return Constants.DriveConstants.invertGyro ? -this.gyro.getRate() : this.gyro.getRate();
  }
  
  public DifferentialDriveKinematics getKinematics() {
    return this.kinematics;
  }
  
  public DifferentialDriveWheelSpeeds getWheelSpeeds() {
    return new DifferentialDriveWheelSpeeds(getLeftEncoderVelocity(), getLeftEncoderVelocity());
  }
  
  public Pose2d getPose() {
    return this.odometry.getPoseMeters();
  }
}
