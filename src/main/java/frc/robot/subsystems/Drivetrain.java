package frc.robot.subsystems;

import java.util.ArrayList;
import java.util.Arrays;

import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.hal.SimDouble;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.hal.simulation.SimDeviceDataJNI;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;

public class Drivetrain extends SubsystemBase {
    private WPI_TalonFX backLeft = new WPI_TalonFX(DriveConstants.backLeftPort);
    private WPI_TalonFX backRight = new WPI_TalonFX(DriveConstants.backRightPort);
    private WPI_TalonFX frontLeft = new WPI_TalonFX(DriveConstants.frontLeftPort);
    private WPI_TalonFX frontRight = new WPI_TalonFX(DriveConstants.frontRightPort);

    // private MotorControllerGroup leftGroup = new MotorControllerGroup((MotorController)frontLeft, (MotorController)backLeft);
    // private MotorControllerGroup rightGroup = new MotorControllerGroup((MotorController)frontRight, (MotorController)backRight);

    private DifferentialDrive differentialDrive = new DifferentialDrive(frontLeft, frontRight);
    private DifferentialDriveKinematics kinematics = new DifferentialDriveKinematics(DriveConstants.trackwidth);
    private DifferentialDriveOdometry odometry = new DifferentialDriveOdometry(new Rotation2d(), new Pose2d());

    private SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(DriveConstants.sVolts, DriveConstants.vVoltSecondsPerMeter, DriveConstants.aVoltSecondsSquaredPerMeter);

    private PIDController leftController = new PIDController(DriveConstants.driveP, DriveConstants.driveI, DriveConstants.driveD);
    private PIDController rightController = new PIDController(DriveConstants.driveP, DriveConstants.driveI, DriveConstants.driveD);

    private AHRS gyro = new AHRS();
    private double headingSnapshot;

    private Field2d field = new Field2d();
    private DifferentialDrivetrainSim drivetrainSim;
    private double gyroSimAngle = 0;
    private double leftEncoderSimVelocity = 0, rightEncoderSimVelocity = 0;
    private double leftEncoderSimPosition = 0, rightEncoderSimPosition = 0;

    public Drivetrain() {
        // Calibrate n reset the gyro
        gyro.calibrate();
        gyro.reset();
        
        // Reset all the drivetrain controllers
        backLeft.configFactoryDefault();
        frontLeft.configFactoryDefault();
        backRight.configFactoryDefault();
        frontRight.configFactoryDefault();

        frontLeft.setInverted(InvertType.None);
        frontLeft.setNeutralMode(NeutralMode.Coast);
        frontLeft.configOpenloopRamp(1/2);
        frontLeft.setSensorPhase(true);
        backLeft.follow(frontLeft);
        backLeft.setNeutralMode(NeutralMode.Coast);
        backLeft.setInverted(InvertType.FollowMaster);
        
        frontRight.setInverted(InvertType.None);
        frontRight.setNeutralMode(NeutralMode.Coast);
        frontRight.configOpenloopRamp(1/2);
        backRight.follow(frontRight);
        backRight.setInverted(InvertType.FollowMaster);
        backRight.setNeutralMode(NeutralMode.Coast);
    
        if (RobotBase.isSimulation()) { // If our robot is simulated
            // This class simulates our drivetrain's motion around the field.
            drivetrainSim = new DifferentialDrivetrainSim(
                LinearSystemId.identifyDrivetrainSystem(
                    DriveConstants.vVoltSecondsPerMeter,
                    DriveConstants.aVoltSecondsSquaredPerMeter,
                    DriveConstants.vVoltSecondsPerRadian,
                    DriveConstants.aVoltSecondsSquaredPerRadian
                ),
                DriveConstants.driveGearbox,
                1./DriveConstants.gearboxRatio,
                DriveConstants.trackwidth,
                DriveConstants.wheelDiameter / 2.0,
                VecBuilder.fill(0, 0, 0.0001, 0.1, 0.1, 0.005, 0.005)
            );
            
        }
        SmartDashboard.putData("Field", field);
    }
    
    @Override
    public void periodic() {
        odometry.update(Rotation2d.fromDegrees(getHeading()), getLeftEncoderPosition(), getRightEncoderPosition());
        field.setRobotPose(new Pose2d(odometry.getPoseMeters().getX(), odometry.getPoseMeters().getY(), odometry.getPoseMeters().getRotation()));

        SmartDashboard.putData("field", field);
        SmartDashboard.putNumber("right pos", getRightEncoderPosition());
        SmartDashboard.putNumber("left pos", getLeftEncoderPosition());
        SmartDashboard.putNumber("gyro", getHeading());
    }

    @Override
    public void simulationPeriodic() {
        drivetrainSim.setInputs(frontLeft.get() * RobotController.getBatteryVoltage(), frontRight.get() * RobotController.getBatteryVoltage());
        drivetrainSim.update(0.020);

        // Encoders
        leftEncoderSimVelocity = (drivetrainSim.getLeftVelocityMetersPerSecond());
        leftEncoderSimPosition = metersToNative(drivetrainSim.getLeftPositionMeters());
        rightEncoderSimVelocity = (drivetrainSim.getRightVelocityMetersPerSecond());
        rightEncoderSimPosition = metersToNative(drivetrainSim.getRightPositionMeters());
        gyroSimAngle = -drivetrainSim.getHeading().getDegrees();
    }

    public void tankDriveVolts(double left, double right) {
        tankDrive(left / 12, right / 12);
    }

    public void tankDrive(double left, double right) {
        differentialDrive.tankDrive(left, right);
    }

    public void arcadeDrive(double throttle, double turn) {
        System.out.println("throttle " + Math.round(throttle*10)/10. + " turn " + Math.round(turn*10)/10. + " pose " + odometry.getPoseMeters());
        differentialDrive.arcadeDrive(throttle, turn);
    }

    public static double nativeToMeters(double counts) { 
        // Helper function to convert native units (encoder counts) to meters for odometry
        double wheelRotations = ((double)counts / DriveConstants.encoderCountsPerRotation) * DriveConstants.gearboxRatio;
        return wheelRotations * (Math.PI * DriveConstants.wheelDiameter);
    }

    public static double metersPerSecondToNative(double metersPerSecond, int hertz) { 
        // Helper function to convert from meters per seconds to encoder units per epoch for PID'ing
        double wheelRotationsPerSecond = metersPerSecond / (Math.PI * DriveConstants.wheelDiameter);
        double motorRotationsPerSecond = wheelRotationsPerSecond * DriveConstants.gearboxRatio;
        return (int)((motorRotationsPerSecond / hertz) * DriveConstants.encoderCountsPerRotation);
    }

    public static double metersPerSecondToNative(double metersPerSecond) { return metersPerSecondToNative(metersPerSecond, 10); } 

    public static double metersToNative(double meters) { 
        // Helper function to convert from meters to native encoder units for PID'ing
        double wheelRotations = meters / (Math.PI * DriveConstants.wheelDiameter);
        double motorRotations = wheelRotations * DriveConstants.gearboxRatio;
        return (int)(motorRotations * DriveConstants.encoderCountsPerRotation);
    }

    public static double nativeToMeters(double meters, boolean inverted) { 
        return inverted ? nativeToMeters(meters) * -1 : nativeToMeters(meters);
    }

    public void setNeutralMode(NeutralMode mode) {
        frontLeft.setNeutralMode(mode);
        frontRight.setNeutralMode(mode);
        backLeft.setNeutralMode(mode);
        backRight.setNeutralMode(mode);
    }
    // This is gross
    public ArrayList<TalonFX> getTalonFXs() { return new ArrayList<> (Arrays.asList(new TalonFX[] { backLeft, backRight, frontLeft, frontRight })); }

    /* Odometry Helper Functions */
    public DifferentialDriveKinematics getKinematics() { return kinematics; }
    public DifferentialDriveWheelSpeeds getWheelSpeeds() { return new DifferentialDriveWheelSpeeds(getLeftEncoderVelocity(), getLeftEncoderVelocity()); }
    public Pose2d getPose() { return odometry.getPoseMeters(); }
    public void resetOdometry() { 
        odometry.resetPosition(new Pose2d(), new Rotation2d()); 
        frontLeft.setSelectedSensorPosition(0);
        frontRight.setSelectedSensorPosition(0);
        gyro.reset();
    }
    public void resetOdometry(Pose2d pose) { 
        odometry.resetPosition(pose, pose.getRotation()); 
        frontLeft.setSelectedSensorPosition(0);
        frontRight.setSelectedSensorPosition(0);
        gyro.reset();
    }

    /* PID Getters */
    public PIDController getLeftController() { return leftController; }
    public PIDController getRightController() { return rightController; }
    public SimpleMotorFeedforward getFeedForward() { return feedforward; }

    /* Sensor Getters */
    public double getLeftEncoderPosition() { 
        if (RobotBase.isSimulation()) { return nativeToMeters(leftEncoderSimPosition); }
        return nativeToMeters((frontLeft.getSelectedSensorPosition() + backLeft.getSelectedSensorPosition())/2., DriveConstants.leftEncoderInverted);
    }
    public double getLeftEncoderVelocity() {
        if (RobotBase.isSimulation()) { return nativeToMeters(leftEncoderSimVelocity); } 
        return nativeToMeters((frontLeft.getSelectedSensorVelocity() + backLeft.getSelectedSensorVelocity())/2. * 10);
    }

    public double getRightEncoderPosition() { 
        if (RobotBase.isSimulation()) { return nativeToMeters(rightEncoderSimPosition); }
        return nativeToMeters((frontRight.getSelectedSensorPosition() + backRight.getSelectedSensorPosition())/2., DriveConstants.rightEncoderInverted); 
    }
    public double getRightEncoderVelocity() {
        if (RobotBase.isSimulation()) { return nativeToMeters(rightEncoderSimVelocity); } 
        return nativeToMeters((frontRight.getSelectedSensorVelocity() + backRight.getSelectedSensorVelocity())/2. * 10); 
    }
    
    public double getHeading() {
        if (RobotBase.isSimulation()) { return DriveConstants.invertGyro ? -gyroSimAngle : gyroSimAngle; } 
        return DriveConstants.invertGyro ? -gyro.getAngle() : gyro.getAngle();
    }
    
    public double getHeadingRate() { return DriveConstants.invertGyro ? -gyro.getRate() : gyro.getRate(); }
    public void takeHeadingSnapshot() { headingSnapshot = getHeading(); }
    public double getHeadingSnapshot() { return headingSnapshot; }

    public void setSlowMode(boolean on) {
        differentialDrive.setMaxOutput(on ? 0.2 : 1);
    }
}