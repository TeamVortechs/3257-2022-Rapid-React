package frc.robot;

import frc.robot.Constants.IOConstants;
import frc.robot.subsystems.Drivetrain;

import com.ctre.phoenix.music.Orchestra;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;

public class RobotContainer {
    private final Drivetrain drivetrain  = new Drivetrain();
    private final Joystick driverController = new Joystick(IOConstants.driverControllerPort);
    private Orchestra orchestra;
    public RobotContainer() {
        // drivetrain.setDefaultCommand(new RunCommand(() -> drivetrain.arcadeDrive(driverController.getLeftStickYValue(), -driverController.getRightStickXValue()), drivetrain)); 
        orchestra = new Orchestra(drivetrain.getTalonFXs());
        orchestra.loadMusic("song10.chrp");
        orchestra.play();
        drivetrain.setDefaultCommand(
            new RunCommand(
                () -> drivetrain.arcadeDrive(
                    driverController.getY(),
                    -driverController.getX()
                ), 
                drivetrain
            )
        ); 
        configureButtonBindings();
    }

    private void configureButtonBindings() {}

    public Command getAutonomousCommand() {
        return null;
    }
}
