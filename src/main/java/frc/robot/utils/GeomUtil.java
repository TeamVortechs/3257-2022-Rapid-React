package frc.robot.utils;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;

public class GeomUtil {
  public static Transform2d transformFromTranslation(Translation2d translation) {
    return new Transform2d(translation, new Rotation2d());
  }
  
  public static Transform2d transformFromTranslation(double x, double y) {
    return new Transform2d(new Translation2d(x, y), new Rotation2d());
  }
  
  public static Transform2d transformFromRotation(Rotation2d rotation) {
    return new Transform2d(new Translation2d(), rotation);
  }
  
  public static Pose2d poseFromTranslation(Translation2d translation) {
    return new Pose2d(translation, new Rotation2d());
  }
  
  public static Pose2d poseFromRotation(Rotation2d rotation) {
    return new Pose2d(new Translation2d(), rotation);
  }
  
  public static Transform2d poseToTransform(Pose2d pose) {
    return new Transform2d(pose.getTranslation(), pose.getRotation());
  }
  
  public static Pose2d transformToPose(Transform2d transform) {
    return new Pose2d(transform.getTranslation(), transform.getRotation());
  }
}
