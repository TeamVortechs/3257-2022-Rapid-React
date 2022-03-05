package frc.robot.utils.control;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

public class AnyButton extends JoystickButton {
  public AnyButton(GenericHID joystick, int buttonNumber) {
    super(joystick, buttonNumber);
  }
  
  public AnyButton(XboxController joystick, XboxJoystick.XboxButton button) {
    super((GenericHID)joystick, button.value);
  }
  
  public AnyButton(XboxJoystick joystick, XboxJoystick.XboxButton button) {
    super((GenericHID)joystick, button.value);
  }
}
