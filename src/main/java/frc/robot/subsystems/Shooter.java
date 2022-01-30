package frc.robot.subsystems;

import java.util.ArrayList;
import java.util.Arrays;

import frc.robot.Constants.ShooterConstants;
import frc.robot.utils.Limelight;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Shooter extends SubsystemBase {
    private Limelight shooterLimelight = new Limelight("limelight-Shoot");

    private WPI_TalonFX frontDrum = new WPI_TalonFX(ShooterConstants.frontDrumPort);
    private WPI_TalonFX backDrum = new WPI_TalonFX(ShooterConstants.backDrumPort);

    private double angle = 0;

    //Constructer
    public Shooter() {
        //set limelight
        shooterLimelight.setPipeline(0);

        // Reset all the controllers
        frontDrum.configFactoryDefault();
        backDrum.configFactoryDefault();

        //Set Drums
        frontDrum.setInverted(InvertType.None);
        frontDrum.setNeutralMode(NeutralMode.Coast);
        frontDrum.configOpenloopRamp(1/2);

        backDrum.setInverted(InvertType.InvertMotorOutput);
        backDrum.setNeutralMode(NeutralMode.Coast);
        backDrum.configOpenloopRamp(1/2);
    }

    //Setters
    public void setAngle(double newAngle) { angle = newAngle; }
    public void setFrontMotor(double speed) { frontDrum.set(ControlMode.PercentOutput, speed); }
    public void setBackMotor(double speed) { backDrum.set(ControlMode.PercentOutput, speed); }
    //Getters
    public double getAngle() { return angle; }
    public ArrayList<TalonFX> getTalonFXs() { return new ArrayList<> (Arrays.asList(new TalonFX[] { frontDrum, backDrum })); }
    public Limelight getShooterLimelight() { return shooterLimelight; }
}
