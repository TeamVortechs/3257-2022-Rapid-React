package frc.robot;

import frc.robot.Constants.IOConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Shooter;
import frc.robot.utils.control.XboxJoystick;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;

public class RobotContainer {
    private final Drivetrain drivetrain = new Drivetrain();
    private final Shooter shooter = new Shooter();
    
    XboxJoystick driverController = new XboxJoystick(IOConstants.driverControllerPort);

    public RobotContainer() {
        drivetrain.setDefaultCommand(new RunCommand(() -> drivetrain.arcadeDrive(driverController.getLeftStickYValue(), -driverController.getRightStickXValue()), drivetrain)); 
        configureButtonBindings();
    }

    private void configureButtonBindings() {
        // Hold X - Aim Toward Target (Uses Limelight)
        driverController.xButton
        .whenHeld(new RunCommand(() -> {
            drivetrain.arcadeDrive(driverController.getLeftStickYValue(), shooter.getShooterLimelight().getYawError() / ShooterConstants.limelightTrackingGain);
        }, drivetrain));
        // Press Y - Change Angle
        driverController.yButton
        .whenPressed(() -> {
            shooter.setAngle(shooter.getAngle() + 0.2);
            if(shooter.getAngle() > 0.15) { shooter.setAngle(0); }
            System.out.println("Shooter's new angle: " + shooter.getAngle());
        });
        // Right Trigger - Shoot
        driverController.rightBumper
        .whenActive(() -> {
            shooter.setFrontMotor(0.8);
            shooter.setBackMotor(0.8 - shooter.getAngle());
        })
        .whenInactive(() -> {
            shooter.setFrontMotor(0);
            shooter.setBackMotor(0);
        });
        
    }

    public Command getAutonomousCommand() {
        return null;
    }
}
