package frc.robot;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;

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
}
