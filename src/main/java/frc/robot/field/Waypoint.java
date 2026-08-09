package frc.robot.field;

import static edu.wpi.first.units.Units.Meters;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.units.measure.Distance;

public record Waypoint(String name, WaypointGroup group, Pose2d bluePose) {

  public Pose2d pose() {
    return AllianceFlipUtil.apply(bluePose);
  }

  public Pose2d approach(Distance offset) {
    return approach(offset.in(Meters));
  }

  public Pose2d approach(double offsetMeters) {
    return FieldConstants.offsetAlongHeading(pose(), offsetMeters);
  }

  public double distanceFrom(Pose2d robotPose) {
    return robotPose.getTranslation().getDistance(pose().getTranslation());
  }
}
