package frc.robot;

import frc.robot.Constants.IOConstants;
import frc.robot.subsystems.Drivetrain;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;

public class RobotContainer {
    private final Drivetrain drivetrain  = new Drivetrain();
    Joystick driverController = new Joystick(IOConstants.driverControllerPort);

    public RobotContainer() {
        // drivetrain.setDefaultCommand(new RunCommand(() -> drivetrain.arcadeDrive(driverController.getLeftStickYValue(), -driverController.getRightStickXValue()), drivetrain)); 
        drivetrain.setDefaultCommand(new RunCommand(() -> drivetrain.arcadeDrive(driverController.getY(), -driverController.getX()), drivetrain)); 
        configureButtonBindings();
    }

    private void configureButtonBindings() {}

    public Command getAutonomousCommand() {
        return null;
    }
}
