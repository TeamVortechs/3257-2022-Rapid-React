package frc.robot.commands;

import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.Trajectory.State;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.Drivetrain;

public class DriveTrajectory extends CommandBase {
    private Trajectory trajectory;
    private RamseteController controller = new RamseteController(DriveConstants.ramseteB, DriveConstants.ramseteZeta);
    private Timer timer = new Timer();
    private Drivetrain drivetrain;
    public DriveTrajectory(Drivetrain drivetrain, Trajectory trajectory) {
        addRequirements(drivetrain);
        this.trajectory = trajectory;
        this.drivetrain = drivetrain;
    }

    @Override
    public void initialize() {
        timer.reset();
        timer.start();
    }

    @Override
    public void execute() {
        State setpoint = trajectory.sample(timer.get());
        ChassisSpeeds chassisSpeeds = controller.calculate(drivetrain.getPose(), setpoint);
        DifferentialDriveWheelSpeeds wheelSpeeds = drivetrain.getKinematics().toWheelSpeeds(chassisSpeeds);
        drivetrain.setWheelSpeeds(wheelSpeeds);
    }

    @Override
    public void end(boolean interrupted) {}

    @Override
    public boolean isFinished() {
        return false;
    }
}
