package frc.robot.commands;

import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.Constants;
import frc.robot.subsystems.Drivetrain;

public class DriveTrajectory extends CommandBase {
  private Trajectory trajectory;
  
  private RamseteController controller = new RamseteController(Constants.DriveConstants.ramseteB, Constants.DriveConstants.ramseteZeta);
  
  private Timer timer = new Timer();
  
  private Drivetrain drivetrain;
  
  public DriveTrajectory(Drivetrain drivetrain, Trajectory trajectory) {
    addRequirements(new Subsystem[] { (Subsystem)drivetrain });
    this.trajectory = trajectory;
    this.drivetrain = drivetrain;
  }
  
  public void initialize() {
    this.timer.reset();
    this.timer.start();
  }
  
  public void execute() {
    Trajectory.State setpoint = this.trajectory.sample(this.timer.get());
    ChassisSpeeds chassisSpeeds = this.controller.calculate(this.drivetrain.getPose(), setpoint);
    DifferentialDriveWheelSpeeds wheelSpeeds = this.drivetrain.getKinematics().toWheelSpeeds(chassisSpeeds);
    this.drivetrain.setWheelSpeeds(wheelSpeeds);
  }
  
  public void end(boolean interrupted) {}
  
  public boolean isFinished() {
    return false;
  }
}
