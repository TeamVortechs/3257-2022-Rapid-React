package frc.robot.utils.control;

import edu.wpi.first.wpilibj.Joystick;

public class ThumbStick {
  Joystick controller;
  
  XboxJoystick.XboxAxis xAxis;
  
  XboxJoystick.XboxAxis yAxis;
  
  double yDeadband = 0.0D;
  
  double xDeadband = 0.0D;
  
  public ThumbStick(Joystick controller, XboxJoystick.XboxAxis xAxis, XboxJoystick.XboxAxis yAxis) {
    this.controller = controller;
    this.xAxis = xAxis;
    this.yAxis = yAxis;
  }
  
  public ThumbStick(Joystick controller, XboxJoystick.XboxAxis xAxis, XboxJoystick.XboxAxis yAxis, double yDeadband, double xDeadband) {
    this(controller, xAxis, yAxis);
    this.yDeadband = Math.abs(yDeadband);
    this.xDeadband = Math.abs(xDeadband);
  }
  
  public double getX() {
    double value = this.controller.getRawAxis(this.xAxis.value);
    return handleDeadband(value, this.xDeadband);
  }
  
  public double getY() {
    double value = this.controller.getRawAxis(this.yAxis.value);
    return handleDeadband(value, this.yDeadband);
  }
  
  public void setXDeadband(double deadband) {
    this.xDeadband = deadband;
  }
  
  public void setYDeadband(double deadband) {
    this.yDeadband = deadband;
  }
  
  public void setDeadband(double xDeadband, double yDeadband) {
    setXDeadband(xDeadband);
    setYDeadband(yDeadband);
  }
  
  public double handleDeadband(double input, double deadband) {
    if (input > -deadband && input < deadband)
      return 0.0D; 
    if (input > 0.0D) {
      double d = (input - deadband) / (1.0D - deadband);
      return d;
    } 
    double scaledInput = (input + deadband) / (1.0D - deadband);
    return scaledInput;
  }
}
