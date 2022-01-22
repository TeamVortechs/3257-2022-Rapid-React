package frc.robot.subsystems;

import frc.robot.Constants.ShooterConstants;
import frc.robot.utils.Limelight;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.PIDSubsystem;

public class Shooter {
    private Limelight shooterLimelight = new Limelight("limelight-Shoot");
    //Constructer
    public Shooter() {
        //super(new PIDController(ShooterConstants.shootP, ShooterConstants.shootI, ShooterConstants.shootD));
        shooterLimelight.setPipeline(0);
    }
    //Getters
    public Limelight getShooterLimelight() { return shooterLimelight; }
}
