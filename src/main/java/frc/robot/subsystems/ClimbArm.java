package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClimbConstants;

public class ClimbArm extends SubsystemBase {
    private WPI_TalonSRX shoulder = new WPI_TalonSRX(ClimbConstants.shoulderPort);
    private WPI_TalonSRX elbow = new WPI_TalonSRX(ClimbConstants.elbowPort);

    private DutyCycleEncoder shoulderEncoder = new DutyCycleEncoder(ClimbConstants.shoulderEncoderPort);
    private PIDController shoulderController = new PIDController(ClimbConstants.shoulderP, ClimbConstants.shoulderI, ClimbConstants.shoulderD);
    private SimpleMotorFeedforward shoulderFeedforward = new SimpleMotorFeedforward(ClimbConstants.shoulderS, ClimbConstants.shoulderV, ClimbConstants.shoulderA);
    
    private DutyCycleEncoder elbowEncoder = new DutyCycleEncoder(ClimbConstants.elbowEncoderPort);
    private PIDController elbowController = new PIDController(ClimbConstants.elbowP, ClimbConstants.elbowI, ClimbConstants.elbowD);
    private SimpleMotorFeedforward elbowFeedforward = new SimpleMotorFeedforward(ClimbConstants.elbowS, ClimbConstants.elbowV, ClimbConstants.elbowA);
    
    public class ArmPose {
        public double shoulderPos;
        public double elbowPos;
        public ArmPose(double shoulderPos, double elbowPos) {
            this.shoulderPos = shoulderPos;
            this.elbowPos = elbowPos;
        }
    }

    public ClimbArm() {
        shoulder.configFactoryDefault();
        shoulder.setNeutralMode(NeutralMode.Brake);
        shoulderEncoder.setDistancePerRotation(0.5);

        elbow.configFactoryDefault();
        elbow.setNeutralMode(NeutralMode.Brake);
        elbowEncoder.setDistancePerRotation(0.5);
    }

    public void setElbowPercentOutput(double out) { elbow.set(ControlMode.PercentOutput, out); }
    public void setShoulderPercentOutput(double out) { shoulder.set(ControlMode.PercentOutput, out); }

    /* Boilerplate trsh */
    public WPI_TalonSRX getShoulderTalon() { return shoulder; }
    public WPI_TalonSRX getElbowTalon() { return elbow; }
    
    public PIDController getShoulderController() { return shoulderController; }
    public PIDController getElbowController() { return elbowController; }
    
    public SimpleMotorFeedforward getShoulderFeedforward() { return shoulderFeedforward; }
    public SimpleMotorFeedforward getElbowFeedforward() { return elbowFeedforward; }

    public DutyCycleEncoder getShoulderEncoder() { return shoulderEncoder; }
    public DutyCycleEncoder getElbowEncoder() { return elbowEncoder; }
}