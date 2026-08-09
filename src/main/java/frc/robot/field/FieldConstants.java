package frc.robot.field;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.measure.Distance;

public final class FieldConstants {

  private FieldConstants() {}

  public static final AprilTagFieldLayout APRIL_TAG_LAYOUT =
      AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);

  public static final double FIELD_LENGTH = APRIL_TAG_LAYOUT.getFieldLength();

  public static final double FIELD_WIDTH = APRIL_TAG_LAYOUT.getFieldWidth();

  public static final Distance BUMPER_FRONT_OFFSET = Inches.of(17.5);

  public static final Distance BUMPER_BACK_OFFSET = Inches.of(17.5);

  public static final Distance DEFAULT_APPROACH_OFFSET = Inches.of(22.0);

  public static boolean isInsideField(Translation2d translation) {
    return translation.getX() >= 0.0
        && translation.getX() <= FIELD_LENGTH
        && translation.getY() >= 0.0
        && translation.getY() <= FIELD_WIDTH;
  }

  public static boolean isInsideField(Pose2d pose) {
    return isInsideField(pose.getTranslation());
  }

  public static Pose2d offsetAlongHeading(Pose2d pose, Distance offset) {
    return offsetAlongHeading(pose, offset.in(Meters));
  }

  public static Pose2d offsetAlongHeading(Pose2d pose, double offsetMeters) {
    return new Pose2d(
        pose.getX() - offsetMeters * pose.getRotation().getCos(),
        pose.getY() - offsetMeters * pose.getRotation().getSin(),
        pose.getRotation());
  }
}
