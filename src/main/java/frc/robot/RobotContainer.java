package frc.robot;

import frc.robot.Constants.IOConstants;
import frc.robot.subsystems.Drivetrain;
import frc.robot.utils.control.XboxJoystick;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;

public class RobotContainer {
    private final Drivetrain drivetrain  = new Drivetrain();
    XboxJoystick driverController = new XboxJoystick(IOConstants.driverControllerPort);

    public RobotContainer() {
        drivetrain.setDefaultCommand(new RunCommand(() -> drivetrain.arcadeDrive(driverController.getLeftStickYValue(), -driverController.getRightStickXValue()), drivetrain)); 
        configureButtonBindings();
    }

    private void configureButtonBindings() {}

    public Command getAutonomousCommand() {
        return null;
    }
}
