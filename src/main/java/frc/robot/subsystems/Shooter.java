package frc.robot.subsystems;

import frc.robot.Constants.ShooterConstants;
import frc.robot.utils.Limelight;

import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Shooter extends SubsystemBase {
    private Limelight shooterLimelight = new Limelight("limelight-Shoot");

    private WPI_TalonFX frontDrum = new WPI_TalonFX(ShooterConstants.frontDrumPort);
    private WPI_TalonFX backDrum = new WPI_TalonFX(ShooterConstants.backDrumPort);

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
    //Getters
    public Limelight getShooterLimelight() { return shooterLimelight; }
}
