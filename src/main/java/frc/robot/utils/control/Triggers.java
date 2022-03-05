package frc.robot.utils.control;

import edu.wpi.first.wpilibj.Joystick;

public class Triggers {
  Joystick controller;
  
  public Triggers(Joystick controller) {
    this.controller = controller;
  }
  
  public double getLeft() {
    return this.controller.getRawAxis(XboxJoystick.XboxAxis.LEFT_TRIGGER.value);
  }
  
  public double getRight() {
    return this.controller.getRawAxis(XboxJoystick.XboxAxis.RIGHT_TRIGGER.value);
  }
  
  public double getTwist() {
    return -getLeft() + getRight();
  }
}
