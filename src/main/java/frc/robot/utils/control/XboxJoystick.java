package frc.robot.utils.control;

import frc.robot.utils.control.AxisButton.ThresholdType;

import edu.wpi.first.wpilibj.Joystick;

public class XboxJoystick extends Joystick {
	public static enum XboxButton {
		A(1), B(2), X(3), Y(4), LEFT_BUMPER(5), RIGHT_BUMPER(6), SELECT(7), START(8), LEFT_STICK(9), RIGHT_STICK(10);
		final int value;

		XboxButton(int value) {
			this.value = value;
		}

		public int getValue() {
			return this.value;
		}
	}

	public static enum XboxAxis {
		LEFT_X(0), LEFT_Y(1), LEFT_TRIGGER(2), RIGHT_TRIGGER(3), RIGHT_X(4), RIGHT_Y(5), DPAD(6);
        final int value;
        
		XboxAxis(int value) {
			this.value = value;
		}

		public int getValue() {
			return this.value;
		}
	}

	public static enum XboxDpad {
		UNPRESSED(-1), UP(0), UP_RIGHT(45), RIGHT(90), DOWN_RIGHT(135), DOWN(180), DOWN_LEFT(225), LEFT(270), UP_LEFT(315);
        final int value;
        
		XboxDpad(int value) {
			this.value = value;
		}

		public int getValue() {
			return this.value;
		}
	}

	public AnyButton xButton = new AnyButton(this, XboxButton.X);
	public AnyButton yButton = new AnyButton(this, XboxButton.Y);
	public AnyButton aButton = new AnyButton(this, XboxButton.A);
	public AnyButton bButton = new AnyButton(this, XboxButton.B);
	public AnyButton rightBumper = new AnyButton(this, XboxButton.RIGHT_BUMPER);
	public AnyButton leftBumper = new AnyButton(this, XboxButton.LEFT_BUMPER);
	public AnyButton startButton = new AnyButton(this, XboxButton.START);
	public AnyButton selectButton = new AnyButton(this, XboxButton.SELECT);
	public AnyButton leftStickButton = new AnyButton(this, XboxButton.LEFT_STICK);
	public AnyButton rightStickButton = new AnyButton(this, XboxButton.RIGHT_STICK);

	public AxisButton leftTriggerButton = new AxisButton(this, XboxAxis.LEFT_TRIGGER, .25, ThresholdType.GREATER_THAN);
	public AxisButton rightTriggerButton = new AxisButton(this, XboxAxis.RIGHT_TRIGGER, .25, ThresholdType.GREATER_THAN);
	public Dpad Dpad = new Dpad(this);

	public ThumbStick leftStick = new ThumbStick(this, XboxAxis.LEFT_X, XboxAxis.LEFT_Y);
	public ThumbStick rightStick = new ThumbStick(this, XboxAxis.RIGHT_X, XboxAxis.RIGHT_Y);

	public Triggers triggers = new Triggers(this);
    
    /**
     * A util class to help with interacting with Xbox Controllers - the WPIlib version doesn't have button objects, and the joystick way is confusing (adapted from 319 Bob Control)
     * @param port controller port
     */
    public XboxJoystick(int port) {
		super(port);
	}

    /**
     * A util class to help with interacting with Xbox Controllers - the WPIlib version doesn't have button objects, and the joystick way is confusing (adapted from 319 Bob Control)
     * @param port controller port
     * @param xDeadband deadband for both stick's x values
     * @param yDeadbandthe deadband for both stick's y values
     */
	public XboxJoystick(int port, double xDeadband, double yDeadband) {
		this(port);
		this.leftStick.setDeadband(xDeadband, yDeadband);
		this.rightStick.setDeadband(xDeadband, yDeadband);
    }
    
	public void setRumble(double leftValue, double rightValue) {
		setRumble(RumbleType.kLeftRumble, leftValue);
		setRumble(RumbleType.kRightRumble, rightValue);
    }

    public double getLeftTriggerValue() { return this.getRawAxis(XboxAxis.LEFT_TRIGGER.value); }

    public double getRightTriggerValue() { return this.getRawAxis(XboxAxis.RIGHT_TRIGGER.value); }

    public double getLeftStickXValue() { return this.getRawAxis(XboxAxis.LEFT_X.value); }

    public double getLeftStickYValue() { return this.getRawAxis(XboxAxis.LEFT_Y.value); }

    public double getRightStickXValue() { return this.getRawAxis(XboxAxis.RIGHT_X.value); }
    
    public double getRightStickYValue() { return this.getRawAxis(XboxAxis.RIGHT_Y.value); }
}
