package frc.robot.utils.control;

import frc.robot.utils.control.XboxJoystick.XboxButton;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

public class AnyButton extends JoystickButton {

	public AnyButton(GenericHID joystick, int buttonNumber) {
		super(joystick, buttonNumber);
	}

	public AnyButton(XboxController joystick, XboxButton button) {
		super(joystick, button.value);
	}

	public AnyButton(XboxJoystick joystick, XboxButton button) {
		super(joystick, button.value);
	}
}
