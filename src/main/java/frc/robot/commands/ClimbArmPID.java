package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.subsystems.ClimbArm;

public class ClimbArmPID extends CommandBase {
  private ClimbArm climber;
  
  private double targetShoulderPos;
  
  private double targetElbowPos;
  
  public ClimbArmPID(double targetShoulderPos, double targetElbowPos, ClimbArm climber) {
    addRequirements(new Subsystem[] { (Subsystem)climber });
    this.climber = climber;
    this.targetElbowPos = targetElbowPos;
    this.targetShoulderPos = targetShoulderPos;
  }
  
  public ClimbArmPID(ClimbArm.ArmPose pose, ClimbArm climber) {
    addRequirements(new Subsystem[] { (Subsystem)climber });
    this.climber = climber;
    this.targetElbowPos = pose.elbowPos;
    this.targetShoulderPos = pose.shoulderPos;
  }
  
  public void initialize() {}
  
  public void execute() {
    this.climber.setElbowPercentOutput(this.climber.getElbowController().calculate(this.climber.getElbowEncoder().getDistance(), this.targetElbowPos));
    this.climber.setShoulderPercentOutput(this.climber.getShoulderController().calculate(this.climber.getShoulderEncoder().getDistance(), this.targetShoulderPos));
  }
  
  public void end(boolean interrupted) {}
  
  public boolean isFinished() {
    return (this.climber.getElbowController().atSetpoint() && this.climber.getShoulderController().atSetpoint());
  }
}
