package frc.robot.field;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;

public final class AllianceFlipUtil {

  private AllianceFlipUtil() {}

  public static boolean shouldFlip() {
    return DriverStation.getAlliance()
        .map(alliance -> alliance == DriverStation.Alliance.Red)
        .orElse(false);
  }

  public static double applyX(double x) {
    return shouldFlip() ? FieldConstants.FIELD_LENGTH - x : x;
  }

  public static double applyY(double y) {
    return shouldFlip() ? FieldConstants.FIELD_WIDTH - y : y;
  }

  public static Translation2d apply(Translation2d translation) {
    return shouldFlip() ? flip(translation) : translation;
  }

  public static Rotation2d apply(Rotation2d rotation) {
    return shouldFlip() ? flip(rotation) : rotation;
  }

  public static Pose2d apply(Pose2d pose) {
    return shouldFlip() ? flip(pose) : pose;
  }

  public static Translation2d flip(Translation2d translation) {
    return new Translation2d(
        FieldConstants.FIELD_LENGTH - translation.getX(),
        FieldConstants.FIELD_WIDTH - translation.getY());
  }

  public static Rotation2d flip(Rotation2d rotation) {
    return rotation.rotateBy(Rotation2d.kPi);
  }

  public static Pose2d flip(Pose2d pose) {
    return new Pose2d(flip(pose.getTranslation()), flip(pose.getRotation()));
  }
}
