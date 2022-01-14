package frc.robot.utils.control;
import frc.robot.utils.control.XboxJoystick.XboxAxis;

import edu.wpi.first.wpilibj.Joystick;

public class Triggers {
	Joystick controller;

	public Triggers(Joystick controller) {
		this.controller = controller;
	}

	public double getLeft() {
		return this.controller.getRawAxis(XboxAxis.LEFT_TRIGGER.value);
	}

	public double getRight() {
		return this.controller.getRawAxis(XboxAxis.RIGHT_TRIGGER.value);
	}

	public double getTwist() {
		return -getLeft() + getRight();
	}
}
