package frc.robot.utils;

import edu.wpi.first.wpilibj.Counter;
import edu.wpi.first.wpilibj.DigitalSource;

public class LIDAR {
    /*
    * Adjust the Calibration Offset to compensate for differences in each unit.
    * We've found this is a reasonably constant value for readings in the 25 cm to 600 cm range.
    * You can also use the offset to zero out the distance between the sensor and edge of the robot.
    */
    private static final int CALIBRATION_OFFSET = -3;
    private Counter counter;
    private boolean verbose = true;

    /**
    * Create an object for a LIDAR-Lite attached to some digital input on the roboRIO
    * 
    * @param source The DigitalInput or DigitalSource where the LIDAR-Lite is attached (ex: new DigitalInput(9))
    */
    public LIDAR (DigitalSource source, boolean verbose) {
	    counter = new Counter(source);
        counter.setMaxPeriod(1.0);
        // Configure for measuring rising to falling pulses
        counter.setSemiPeriodMode(true);
        counter.reset();
        this.verbose = verbose;
    }

    /**
    * Take a measurement and return the distance in cm
    * 
    * @return Distance in cm
    */
    public double getDistance() {
	    double cm;
        /* 
        * If we haven't seen the first rising to falling pulse, then we have no measurement.
	    * This happens when there is no LIDAR-Lite plugged in, btw.
	    */
	    if (counter.get() < 1) {
		    if (verbose) {
                System.out.println("LidarLitePWM: waiting for distance measurement");
		    }
		    return 0;
        }
        
        /* 
        * getPeriod returns time in seconds. The hardware resolution is microseconds.
        * The LIDAR-Lite unit sends a high signal for 10 microseconds per cm of distance.
        */
        cm = (counter.getPeriod() * 1000000.0 / 10.0) + CALIBRATION_OFFSET;
        //System.out.println("LidarLitePWM: Got " + cm + "cm");
	    return cm;
    }
}