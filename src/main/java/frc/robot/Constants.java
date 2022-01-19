package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import frc.robot.utils.GeomUtil;

public final class Constants {
    public static class DriveConstants {
        /* Drive Ports */
        public static int backLeftPort = 3;
        public static int backRightPort = 1;
        public static int frontLeftPort = 4;
        public static int frontRightPort = 2;

        /* PID Controller Gains */
        public static double driveP = 0.0147;
        public static double driveI = 0;
        public static double driveD = 0;
        public static double driveF = 0;

        public static double talonFXP = 0.1;
        public static double talonFXI = 0.001;
        public static double talonFXD = 5;
        public static double talonFXF = 0;

        public static double ramseteB = 2;
        public static double ramseteZeta = .7;

        public static double trackingGain = 0.1;
        public static double shootingTrackingGain = 0.05;
        public static double shootingTrackingFeedForward = 1;
        
        public static double turnPIDTolerance;
        
        /* Other Settings */
        public static boolean leftEncoderInverted = true;
        public static boolean rightEncoderInverted = false;

        /* Feedforward Gains (Get these by using the WPIlib characterization tool) */
        public static final double sVolts = 0.7;
        public static final double vVoltSecondsPerMeter = 1.98;
        public static final double aVoltSecondsSquaredPerMeter = 0.2;

        public static final double vVoltSecondsPerRadian = 1.5;
        public static final double aVoltSecondsSquaredPerRadian = 0.3;
        
        public static final DCMotor driveGearbox = DCMotor.getFalcon500(2);

        /* Physical Constants */
        public static double wheelDiameter = Units.inchesToMeters(4);
        public static double encoderCountsPerRotation = 2048/4;
        public static double gearboxRatio = 0.1;
        public static double trackwidth = Units.feetToMeters(1.9022538253313759);
        public static boolean invertGyro = true; // Set to counterclockwise is positive
    }
    
    public static class IOConstants {
        /* Controller Ports */
        public static int driverControllerPort = 0;
        public static int operatorControllerPort = 1;
    }

    /**
     * YOINKED FROM 6238 MECHANICAL ADVANTAGE
     */
    public static class FieldConstants {

        // Field dimensions
        public static final double fieldLength = Units.inchesToMeters(54.0 * 12.0);
        public static final double fieldWidth = Units.inchesToMeters(27.0 * 12.0);
        public static final double hangarLength = Units.inchesToMeters(128.75);
        public static final double hangarWidth = Units.inchesToMeters(116.0);

        // Vision target
        public static final double visionTargetDiameter = Units.inchesToMeters(4.0 * 12.0 + 5.375);
        public static final double visionTargetHeightLower = Units.inchesToMeters(8.0 * 12 + 5.625); // Bottom of tape
        public static final double visionTargetHeightUpper = visionTargetHeightLower + Units.inchesToMeters(2.0); // Top of tape

        // Dimensions of hub and tarmac
        public static final Rotation2d centerLineAngle = Rotation2d.fromDegrees(66.0);
        public static final Translation2d hubCenter = new Translation2d(fieldLength / 2.0, fieldWidth / 2.0);
        public static final double tarmacDiameter = Units.inchesToMeters(219.25); // Inner diameter
        public static final double tarmacFullSideLength = tarmacDiameter * (Math.sqrt(2.0) - 1.0); // If the tarmac formed a full octagon
        public static final double tarmacMarkedSideLength = Units.inchesToMeters(82.83); // Length of tape marking outside of tarmac
        public static final double tarmacMissingSideLength = tarmacFullSideLength - tarmacMarkedSideLength; // Length removed b/c of corner cutoff

        // Reference rotations (angle from hub to each reference point)
        public static final Rotation2d referenceARotation = Rotation2d.fromDegrees(180.0).minus(centerLineAngle).plus(Rotation2d.fromDegrees(360.0 / 16.0));
        public static final Rotation2d referenceBRotation = referenceARotation.rotateBy(Rotation2d.fromDegrees(360.0 / 8.0));
        public static final Rotation2d referenceCRotation = referenceBRotation.rotateBy(Rotation2d.fromDegrees(360.0 / 8.0));
        public static final Rotation2d referenceDRotation = referenceCRotation.rotateBy(Rotation2d.fromDegrees(360.0 / 8.0));

        // Reference points (centered of the sides of the tarmac if they formed a complete octagon)
        public static final Pose2d referenceA = new Pose2d(hubCenter, referenceARotation).transformBy(GeomUtil.transformFromTranslation(tarmacDiameter / 2.0, 0.0));
        public static final Pose2d referenceB = new Pose2d(hubCenter, referenceBRotation).transformBy(GeomUtil.transformFromTranslation(tarmacDiameter / 2.0, 0.0));
        public static final Pose2d referenceC = new Pose2d(hubCenter, referenceCRotation).transformBy(GeomUtil.transformFromTranslation(tarmacDiameter / 2.0, 0.0));
        public static final Pose2d referenceD = new Pose2d(hubCenter, referenceDRotation).transformBy(GeomUtil.transformFromTranslation(tarmacDiameter / 2.0, 0.0));

        // Cargo points
        public static final double cornerToCargoY = Units.inchesToMeters(15.56);
        public static final double referenceToCargoY = (tarmacFullSideLength / 2.0) - cornerToCargoY;
        public static final double referenceToCargoX = Units.inchesToMeters(40.44);
        public static final Pose2d cargoA = referenceA.transformBy(GeomUtil.transformFromTranslation(referenceToCargoX, -referenceToCargoY));
        public static final Pose2d cargoB = referenceA.transformBy(GeomUtil.transformFromTranslation(referenceToCargoX, referenceToCargoY));
        public static final Pose2d cargoC = referenceB.transformBy(GeomUtil.transformFromTranslation(referenceToCargoX, referenceToCargoY));
        public static final Pose2d cargoD = referenceC.transformBy(GeomUtil.transformFromTranslation(referenceToCargoX, -referenceToCargoY));
        public static final Pose2d cargoE = referenceD.transformBy(GeomUtil.transformFromTranslation(referenceToCargoX, -referenceToCargoY));
        public static final Pose2d cargoF = referenceD.transformBy(GeomUtil.transformFromTranslation(referenceToCargoX, referenceToCargoY));
    }
}
