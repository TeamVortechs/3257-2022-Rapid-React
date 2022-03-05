package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

public class Robot extends TimedRobot {
  private Command autonomousCommand;
  
  public void robotInit() {
    new RobotContainer();
  }
  
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
  }
  
  public void disabledInit() {}
  
  public void disabledPeriodic() {}
  
  public void autonomousInit() {
    if (this.autonomousCommand != null)
      this.autonomousCommand.schedule(); 
  }
  
  public void autonomousPeriodic() {}
  
  public void teleopInit() {
    if (this.autonomousCommand != null)
      this.autonomousCommand.cancel(); 
  }
  
  public void teleopPeriodic() {}
  
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
  }
  
  public void testPeriodic() {}
}
