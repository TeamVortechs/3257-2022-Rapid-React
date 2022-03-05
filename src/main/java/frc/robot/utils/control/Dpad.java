package frc.robot.utils.control;

import edu.wpi.first.wpilibj.Joystick;

public class Dpad {
  public final Joystick joy;
  
  public AxisButton Up;
  
  public AxisButton Down;
  
  public AxisButton Left;
  
  public AxisButton Right;
  
  public AxisButton UpLeft;
  
  public AxisButton UpRight;
  
  public AxisButton DownLeft;
  
  public AxisButton DownRight;
  
  public Dpad(Joystick joystick) {
    this.joy = joystick;
    this.Up = new AxisButton(this.joy, XboxJoystick.XboxAxis.DPAD, XboxJoystick.XboxDpad.UP.value, AxisButton.ThresholdType.POV);
    this.Down = new AxisButton(this.joy, XboxJoystick.XboxAxis.DPAD, XboxJoystick.XboxDpad.DOWN.value, AxisButton.ThresholdType.POV);
    this.Left = new AxisButton(this.joy, XboxJoystick.XboxAxis.DPAD, XboxJoystick.XboxDpad.LEFT.value, AxisButton.ThresholdType.POV);
    this.Right = new AxisButton(this.joy, XboxJoystick.XboxAxis.DPAD, XboxJoystick.XboxDpad.RIGHT.value, AxisButton.ThresholdType.POV);
    this.UpLeft = new AxisButton(this.joy, XboxJoystick.XboxAxis.DPAD, XboxJoystick.XboxDpad.UP_LEFT.value, AxisButton.ThresholdType.POV);
    this.UpRight = new AxisButton(this.joy, XboxJoystick.XboxAxis.DPAD, XboxJoystick.XboxDpad.UP_RIGHT.value, AxisButton.ThresholdType.POV);
    this.DownLeft = new AxisButton(this.joy, XboxJoystick.XboxAxis.DPAD, XboxJoystick.XboxDpad.DOWN_LEFT.value, AxisButton.ThresholdType.POV);
    this.DownRight = new AxisButton(this.joy, XboxJoystick.XboxAxis.DPAD, XboxJoystick.XboxDpad.DOWN_RIGHT.value, AxisButton.ThresholdType.POV);
  }
  
  public double getValue() {
    return this.joy.getRawAxis(XboxJoystick.XboxAxis.DPAD.value);
  }
}
