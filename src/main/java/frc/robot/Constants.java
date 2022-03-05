package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import frc.robot.utils.GeomUtil;

public final class Constants {
  public static class DriveConstants {
    public static int frontLeftPort = 1;
    
    public static int backLeftPort = 2;
    
    public static int frontRightPort = 3;
    
    public static int backRightPort = 4;
    
    public static double talonFXP = 0.2D;
    
    public static double talonFXI = 0.0D;
    
    public static double talonFXD = 2.0D;
    
    public static double talonFXF = 0.0D;
    
    public static double ramseteB = 2.0D;
    
    public static double ramseteZeta = 0.7D;
    
    public static boolean leftEncoderInverted = true;
    
    public static boolean rightEncoderInverted = false;
    
    public static final double sVolts = 0.7D;
    
    public static final double vVoltSecondsPerMeter = 1.98D;
    
    public static final double aVoltSecondsSquaredPerMeter = 0.2D;
    
    public static final double vVoltSecondsPerRadian = 1.5D;
    
    public static final double aVoltSecondsSquaredPerRadian = 0.3D;
    
    public static final DCMotor driveGearbox = DCMotor.getFalcon500(2);
    
    public static double wheelDiameter = Units.inchesToMeters(4.0D);
    
    public static double gearboxRatio = 0.1D;
    
    public static double trackwidth = Units.feetToMeters(1.9022538253313759D);
    
    public static boolean invertGyro = true;
  }
  
  public static class IOConstants {
    public static int driverControllerPort = 0;
    
    public static int operatorControllerPort = 1;
  }
  
  public static class ShooterConstants {
    public static int upperFlywheelPort = 8;
    
    public static int lowerFlywheelPort = 6;
    
    public static double flywheelV = 0.023D;
    
    public static double flywheelA = 0.001D;
    
    public static double spinupRadPerSec = Units.rotationsPerMinuteToRadiansPerSecond(500.0D);
    
    public static double flywheelMOI = 0.001D;
    
    public static double flywheelGearing = 1.0D;
  }
  
  public static class ClimbConstants {
    public static int shoulderPort = 0;
    
    public static int elbowPort = 1;
    
    public static int shoulderEncoderPort = 0;
    
    public static int elbowEncoderPort = 0;
    
    public static int jointEncoderCountsPerRotation = 8192;
    
    public static double shoulderP = 0.1D;
    
    public static double shoulderI = 0.0D;
    
    public static double shoulderD = 0.0D;
    
    public static double shoulderS = 0.1D;
    
    public static double shoulderV = 0.0D;
    
    public static double shoulderA = 0.0D;
    
    public static double elbowP = 0.1D;
    
    public static double elbowI = 0.0D;
    
    public static double elbowD = 0.0D;
    
    public static double elbowS = 0.1D;
    
    public static double elbowV = 0.0D;
    
    public static double elbowA = 0.0D;
  }
  
  public static class IntestineConstants {
    public static int intakeSolenoidPort = 0;
    
    public static int intakeSparkPort = 2;
    
    public static int magazinePort = 11;
    
    public static int magazineSwitchPort = 0;
  }
  
  public static class FieldConstants {
    public static final double fieldLength = Units.inchesToMeters(648.0D);
    
    public static final double fieldWidth = Units.inchesToMeters(324.0D);
    
    public static final double hangarLength = Units.inchesToMeters(128.75D);
    
    public static final double hangarWidth = Units.inchesToMeters(116.0D);
    
    public static final double visionTargetDiameter = Units.inchesToMeters(53.375D);
    
    public static final double visionTargetHeightLower = Units.inchesToMeters(101.625D);
    
    public static final double visionTargetHeightUpper = visionTargetHeightLower + Units.inchesToMeters(2.0D);
    
    public static final Rotation2d centerLineAngle = Rotation2d.fromDegrees(66.0D);
    
    public static final Translation2d hubCenter = new Translation2d(fieldLength / 2.0D, fieldWidth / 2.0D);
    
    public static final double tarmacDiameter = Units.inchesToMeters(219.25D);
    
    public static final double tarmacFullSideLength = tarmacDiameter * (Math.sqrt(2.0D) - 1.0D);
    
    public static final double tarmacMarkedSideLength = Units.inchesToMeters(82.83D);
    
    public static final double tarmacMissingSideLength = tarmacFullSideLength - tarmacMarkedSideLength;
    
    public static final Rotation2d referenceARotation = Rotation2d.fromDegrees(180.0D).minus(centerLineAngle).plus(Rotation2d.fromDegrees(22.5D));
    
    public static final Rotation2d referenceBRotation = referenceARotation.rotateBy(Rotation2d.fromDegrees(45.0D));
    
    public static final Rotation2d referenceCRotation = referenceBRotation.rotateBy(Rotation2d.fromDegrees(45.0D));
    
    public static final Rotation2d referenceDRotation = referenceCRotation.rotateBy(Rotation2d.fromDegrees(45.0D));
    
    public static final Pose2d referenceA = (new Pose2d(hubCenter, referenceARotation)).transformBy(GeomUtil.transformFromTranslation(tarmacDiameter / 2.0D, 0.0D));
    
    public static final Pose2d referenceB = (new Pose2d(hubCenter, referenceBRotation)).transformBy(GeomUtil.transformFromTranslation(tarmacDiameter / 2.0D, 0.0D));
    
    public static final Pose2d referenceC = (new Pose2d(hubCenter, referenceCRotation)).transformBy(GeomUtil.transformFromTranslation(tarmacDiameter / 2.0D, 0.0D));
    
    public static final Pose2d referenceD = (new Pose2d(hubCenter, referenceDRotation)).transformBy(GeomUtil.transformFromTranslation(tarmacDiameter / 2.0D, 0.0D));
    
    public static final double cornerToCargoY = Units.inchesToMeters(15.56D);
    
    public static final double referenceToCargoY = tarmacFullSideLength / 2.0D - cornerToCargoY;
    
    public static final double referenceToCargoX = Units.inchesToMeters(40.44D);
    
    public static final Pose2d cargoA = referenceA.transformBy(GeomUtil.transformFromTranslation(referenceToCargoX, -referenceToCargoY));
    
    public static final Pose2d cargoB = referenceA.transformBy(GeomUtil.transformFromTranslation(referenceToCargoX, referenceToCargoY));
    
    public static final Pose2d cargoC = referenceB.transformBy(GeomUtil.transformFromTranslation(referenceToCargoX, referenceToCargoY));
    
    public static final Pose2d cargoD = referenceC.transformBy(GeomUtil.transformFromTranslation(referenceToCargoX, -referenceToCargoY));
    
    public static final Pose2d cargoE = referenceD.transformBy(GeomUtil.transformFromTranslation(referenceToCargoX, -referenceToCargoY));
    
    public static final Pose2d cargoF = referenceD.transformBy(GeomUtil.transformFromTranslation(referenceToCargoX, referenceToCargoY));
  }
}
