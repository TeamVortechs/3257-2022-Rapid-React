package frc.robot.utils;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;

public class Limelight {
    private NetworkTable tableInstance;

    // Used to provide an empirical start point from which the displacement is based on
    private double lastHeading;
    
    /**
     * A util class to help with interacting with limelights (the network table way is cool and all, but I like classes)
     * @param name the name of the limelight (to get the network table entry)
     */
    public Limelight(String name) {
        tableInstance = NetworkTableInstance.getDefault().getTable(name);
        setLightState(1);
    }

    /**
     * Sets the limelights ledMode:
     * (0: Default to pipeline,
     * 1: Force Off,
     * 2: Force Blink,
     * 3: Force On)
     * @param mode to set the limelight leds
     */
    public void setLightState(int mode) {
        tableInstance.getEntry("ledMode").setNumber(mode);
    }

    public void setPipeline(int pipeline) {
        tableInstance.getEntry("pipeline").setNumber(pipeline);
    }

    public double getYawError() {
        return tableInstance.getEntry("tx").getDouble(0);
    }

    public double getYawAngle() {
        return tableInstance.getEntry("tx").getDouble(0) + lastHeading;
    }

    public double getPitchError() {
        return tableInstance.getEntry("tx").getDouble(0);
    }

    public double getTargetArea() {
        return tableInstance.getEntry("ta").getDouble(0);
    }

    public boolean hasTarget() {
        return tableInstance.getEntry("tv").getDouble(0) == 1;
    }

    public void setCurrentHeading(double heading) {
        lastHeading = heading;
    }
}
