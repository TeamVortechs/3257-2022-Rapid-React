package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.math.Nat;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.LinearQuadraticRegulator;
import edu.wpi.first.math.estimator.KalmanFilter;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.LinearSystemLoop;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.ShooterConstants;

public class Shooter extends SubsystemBase {
    private WPI_TalonFX flywheel1TalonFX = new WPI_TalonFX(ShooterConstants.flywheel1Port);
    private WPI_TalonFX flywheel2TalonFX = new WPI_TalonFX(ShooterConstants.flywheel2Port);
    private LinearSystem<N1, N1, N1> flywheelPlant = LinearSystemId.identifyVelocitySystem(ShooterConstants.flywheelV, ShooterConstants.flywheelA);

    private KalmanFilter<N1, N1, N1> flywheel1Observer = new KalmanFilter<>(
        Nat.N1(), Nat.N1(),
        flywheelPlant,
        VecBuilder.fill(3.0),
        VecBuilder.fill(0.01),
        0.02
    );
    private KalmanFilter<N1, N1, N1> flywheel2Observer = new KalmanFilter<>(
        Nat.N1(), Nat.N1(),
        flywheelPlant,
        VecBuilder.fill(3.0),
        VecBuilder.fill(0.01),
        0.02
    );

    private LinearQuadraticRegulator<N1, N1, N1> flywheel1Controller = new LinearQuadraticRegulator<>(
        flywheelPlant,
        VecBuilder.fill(8.0),
        VecBuilder.fill(12.0),
        0.02
    );
    private LinearQuadraticRegulator<N1, N1, N1> flywheel2Controller = new LinearQuadraticRegulator<>(
        flywheelPlant,
        VecBuilder.fill(8.0),
        VecBuilder.fill(12.0),
        0.02
    );

    private LinearSystemLoop<N1, N1, N1> flywheel1Loop = new LinearSystemLoop<>(flywheelPlant, flywheel1Controller, flywheel1Observer, 12.0, 0.020);
    private LinearSystemLoop<N1, N1, N1> flywheel2Loop = new LinearSystemLoop<>(flywheelPlant, flywheel2Controller, flywheel2Observer, 12.0, 0.020);
    private boolean flywheelSpinning = false;

    public Shooter() {
        
    }

    @Override
    public void periodic() {
        flywheel1Loop.setNextR(VecBuilder.fill(flywheelSpinning ? ShooterConstants.spinupRadPerSec : 0.));
        flywheel2Loop.setNextR(VecBuilder.fill(flywheelSpinning ? ShooterConstants.spinupRadPerSec : 0.));

        flywheel1Loop.correct(VecBuilder.fill(2. * Math.PI * flywheel1TalonFX.getSelectedSensorVelocity() / DriveConstants.encoderCountsPerRotation));
        flywheel2Loop.correct(VecBuilder.fill(2. * Math.PI * flywheel2TalonFX.getSelectedSensorVelocity() / DriveConstants.encoderCountsPerRotation));

        flywheel1Loop.predict(0.020);
        flywheel2Loop.predict(0.020);

        flywheel1TalonFX.setVoltage(flywheel1Loop.getU(0));
        flywheel2TalonFX.setVoltage(flywheel2Loop.getU(0));
    }
}
