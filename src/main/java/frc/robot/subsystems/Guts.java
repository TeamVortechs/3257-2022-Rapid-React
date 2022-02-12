package frc.robot.subsystems;

import java.util.ArrayList;
import java.util.Arrays;

import frc.robot.Constants.GutsConstants;
import frc.robot.utils.Limelight;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Guts extends SubsystemBase {
    private CANSparkMax magazineNeo = new CANSparkMax(GutsConstants.magPort, MotorType.kBrushless);

    //Constructer
    public Guts() {

    }

    //Setters
    public void setMagazine(double speed) { magazineNeo.set(speed); }
    //Getters
    
}
