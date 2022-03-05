package frc.robot;

import edu.wpi.first.wpilibj.RobotBase;
import java.util.function.Supplier;

public final class Main {
  public static void main(String... args) {
    RobotBase.startRobot(Robot::new);
  }
}
