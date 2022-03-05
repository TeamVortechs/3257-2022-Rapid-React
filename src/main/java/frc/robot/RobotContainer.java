package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.subsystems.Shooter;
import frc.robot.utils.control.XboxJoystick;

public class RobotContainer {
    private final Drivetrain drivetrain  = new Drivetrain();
    private final Intestines intestines  = new Intestines();
    private final Shooter shooter = new Shooter();
  
    private boolean intestinesOverride = false;
  
    private final XboxJoystick driverController = new XboxJoystick(Constants.IOConstants.driverControllerPort);
  
    public RobotContainer() {
        drivetrain.setDefaultCommand(
            new RunCommand(
                () -> drivetrain.arcadeDrive(
                    -driverController.getX()/3.,
                    -driverController.getY()/3.
                ), 
                drivetrain
            )
        ); 
        //mmmmmmmm autofeed nicee
        intestines.setDefaultCommand(
            new RunCommand(
                () ->  {
                    if (!intestinesOverride)
                        intestines.setMagazinePercent(intestines.isBallInQueue() ? 0.3 : 0);
                    // intestines.setMagazinePercent(0.1);
                    // intestines.setIntakePercent(-0.6);
                }, 
                intestines
            )
        ); 
        configureButtonBindings();
    }
  
    private void configureButtonBindings() {
        driverController.leftBumper
            .whenActive(new InstantCommand(() -> { intestines.setMagazinePercent(.5); intestinesOverride = true; }, intestines))
            .whenInactive(new InstantCommand(() -> { intestines.setMagazinePercent(0); intestinesOverride = false; }, intestines));
        
        driverController.leftTriggerButton
            .whenActive(new InstantCommand(() -> { intestines.setMagazinePercent(-.5); intestines.setIntakePercent(.3); intestinesOverride = true; }, intestines))
            .whenInactive(new InstantCommand(() -> { intestines.setMagazinePercent(0); intestines.setIntakePercent(0); intestinesOverride = false; }, intestines));
        this.driverController.rightBumper
            .whenActive((Command)new InstantCommand(() -> this.shooter.setShooterSpeeds(50.0D, 50.0D), new Subsystem[] { (Subsystem)this.shooter })).whenInactive((Command)new InstantCommand(() -> this.shooter.setShooterSpeeds(0.0D, 0.0D), new Subsystem[] { (Subsystem)this.shooter }));
    }
  
  public Command getAutonomousCommand() {
    return null;
    // return new RunCommand(()->{
        //     // drivetrain.setWheelSpeeds(new DifferentialDriveWheelSpeeds(100, 100));
        //     // System.out.println(drivetrain.getWheelSpeeds());
        //     intestines.setMagazinePercent(05);
        // }, drivetrain);
        // return new SequentialCommandGroup(
        //     new InstantCommand(()->{intestines.setMagazinePercent(0.5); System.out.println("FUCKKK");}),
        //     new WaitCommand(0.5),
        //     new InstantCommand(()->{intestines.setMagazinePercent(0);})
        // );
  }
}
