package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import edu.wpi.first.math.controller.BangBangController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.utils.Conversions;

public class Shooter extends SubsystemBase {
  private WPI_TalonFX lowerFlywheel = new WPI_TalonFX(Constants.ShooterConstants.lowerFlywheelPort);
  
  private WPI_TalonFX upperFlywheel = new WPI_TalonFX(Constants.ShooterConstants.upperFlywheelPort);
  
  private BangBangController controller = new BangBangController();
  
  private double lowerRPM;
  
  private double upperRPM;
  
  public Shooter() {
    this.lowerFlywheel.configFactoryDefault();
    this.upperFlywheel.configFactoryDefault();
    this.lowerFlywheel.setInverted(InvertType.InvertMotorOutput);
    this.lowerFlywheel.setNeutralMode(NeutralMode.Coast);
    this.upperFlywheel.setInverted(InvertType.None);
    this.upperFlywheel.setNeutralMode(NeutralMode.Coast);
  }
  
  public void setShooterSpeeds(double lowerRPM, double upperRPM) {
    this.lowerRPM = lowerRPM;
    this.upperRPM = upperRPM;
  }
  
  public void periodic() {
    if (this.lowerRPM != 0.0D) {
      this.lowerFlywheel.set(this.controller.calculate(Conversions.TalonFXConversions.Native2RPM(this.lowerFlywheel.getSelectedSensorVelocity()), this.lowerRPM) * 0.4D);
    } else {
      this.lowerFlywheel.set(0.0D);
    } 
    if (this.upperRPM != 0.0D) {
      this.upperFlywheel.set(this.controller.calculate(Conversions.TalonFXConversions.Native2RPM(this.upperFlywheel.getSelectedSensorVelocity()), this.upperRPM) * 0.4D);
    } else {
      this.upperFlywheel.set(0.0D);
    } 
    SmartDashboard.putNumber("lower flywheel rpm", Conversions.TalonFXConversions.Native2RPM(this.lowerFlywheel.getSelectedSensorVelocity()));
    SmartDashboard.putNumber("upper flywheel rpm", Conversions.TalonFXConversions.Native2RPM(this.upperFlywheel.getSelectedSensorVelocity()));
    SmartDashboard.putNumber("lower flywheel target", this.lowerRPM);
    SmartDashboard.putNumber("upper flywheel target", this.upperRPM);
  }
}
