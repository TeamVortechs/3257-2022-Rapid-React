package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.subsystems.Shooter;
import frc.robot.utils.control.XboxJoystick;

public class RobotContainer {
  private final Shooter shooter = new Shooter();
  
  private boolean intestinesOverride = false;
  
  private final XboxJoystick driverController = new XboxJoystick(Constants.IOConstants.driverControllerPort);
  
  public RobotContainer() {
    configureButtonBindings();
  }
  
  private void configureButtonBindings() {
    this.driverController.rightBumper
      .whenActive((Command)new InstantCommand(() -> this.shooter.setShooterSpeeds(50.0D, 50.0D), new Subsystem[] { (Subsystem)this.shooter })).whenInactive((Command)new InstantCommand(() -> this.shooter.setShooterSpeeds(0.0D, 0.0D), new Subsystem[] { (Subsystem)this.shooter }));
  }
  
  public Command getAutonomousCommand() {
    return null;
  }
}
