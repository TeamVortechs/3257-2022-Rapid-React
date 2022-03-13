// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.PIDSubsystem;
import com.revrobotics.CANSparkMaxLowLevel;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;

import frc.robot.Constants;

public class Intake extends PIDSubsystem {

  private CANSparkMax intakeLift = new  CANSparkMax(Constants.IntestineConstants.intakeLiftSparkPort, CANSparkMaxLowLevel.MotorType.kBrushless);
  private RelativeEncoder intakeEncoder;

  public Intake() {
        super(new PIDController(0, 0, 0));
        intakeEncoder = intakeLift.getEncoder();
        SmartDashboard.putNumber("intake winch pos", getMeasurement());
    }

  @Override
  public void useOutput(double output, double setpoint) {
    // Use the output here

    intakeLift.set(output);

  }

  @Override
  public double getMeasurement() {
    return intakeEncoder.getPosition();
  }


  public void setIntakeOn(boolean up){
    if(up){
      setSetpoint(1);
      System.out.println("intake up");
    }
    else{
      setSetpoint(0);
      System.out.println("intake down");
    }
  }
}
