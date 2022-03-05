package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Intestines extends SubsystemBase {
  private Spark intakeMotor = new Spark(Constants.IntestineConstants.intakeSparkPort);
  
  private boolean isIntakeActuated = false;
  
  private CANSparkMax magazineMotor = new CANSparkMax(Constants.IntestineConstants.magazinePort, CANSparkMaxLowLevel.MotorType.kBrushless);
  
  private DigitalInput magazineSwitch = new DigitalInput(Constants.IntestineConstants.magazineSwitchPort);
  
  public void actuateIntake(boolean on) {
    this.isIntakeActuated = on;
  }
  
  public void setIntakePercent(double percent) {
    this.intakeMotor.set(percent);
  }
  
  public boolean getIntakeState() {
    return this.isIntakeActuated;
  }
  
  public boolean isBallInQueue() {
    return !this.magazineSwitch.get();
  }
  
  public void setMagazinePercent(double percent) {
    this.magazineMotor.set(percent);
  }
  
  public void periodic() {
    SmartDashboard.putBoolean("Magazine Switch", isBallInQueue());
  }
}
