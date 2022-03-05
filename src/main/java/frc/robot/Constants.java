package frc.robot;

import edu.wpi.first.math.util.Units;

public final class Constants {
    public static class DriveConstants {
        /* Drive Ports */
        public static int backLeftPort = 3;
        public static int backRightPort = 5; //1
        public static int frontLeftPort = 4;
        public static int frontRightPort = 6; //2

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
        public static double s = 0.99;
        public static double v = 7.31;
        public static double a = 0.443;
        
        /* Physical Constants */
        public static double wheelDiameter = Units.inchesToMeters(4);
        public static double encoderCountsPerRotation = 2048;
        public static double gearboxRatio = 0.1;
        public static double trackwidth = Units.feetToMeters(1.9022538253313759);
        public static boolean invertGyro = true; // Set to counterclockwise is positive
    }
    
    public static class ShooterConstants {
        /* Shoot Ports */
        public static int frontDrumPort = 1;
        public static int backDrumPort = 2;

        /*Limelight Gain*/
        public static double limelightTrackingGain = 30;
    }

    public static class GutsConstants {
        /* Guts Ports */
        public static int magPort = 5;
    }

    public static class IntakeConstants {
        /* Guts Ports */
        public static int mainIntakePort = 6;
        //public static int dropIntakePort = 7;
    }

    public static class IOConstants {
        /* Controller Ports */
        public static int driverControllerPort = 0;
        public static int operatorControllerPort = 1;
    }
}
