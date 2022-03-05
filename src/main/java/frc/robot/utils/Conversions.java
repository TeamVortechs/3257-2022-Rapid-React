package frc.robot.utils;

public class Conversions {
  public static class TalonFXConversions {
    public static double RPM2Native(double rpm) {
      return rpm * 2048.0D / 10.0D / 60.0D;
    }
    
    public static double Native2RPM(double units) {
      return units / 2048.0D * 10.0D;
    }
    
    public static double Native2Rotations(double units) {
      return units / 2048.0D * 10.0D;
    }
    
    public static double Rotations2Native(double rotations) {
      return rotations * 2048.0D;
    }
  }
}
