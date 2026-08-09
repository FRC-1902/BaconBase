package frc.robot.field;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import java.util.ArrayList;
import java.util.Collections;
import java.util.Comparator;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;
import java.util.Optional;

public final class Waypoints {

  private Waypoints() {}

  private static final Map<String, Waypoint> REGISTRY = new LinkedHashMap<>();

  static {
    register(
        "Example Waypoint (Type 1)",
        WaypointGroup.PLACEHOLDER,
        new Pose2d(3.0, 4.0, Rotation2d.kZero));
    register(
        "Example Waypoint (Type 2)",
        WaypointGroup.PLACEHOLDER_2,
        new Pose2d(1.5, 7.0, Rotation2d.k180deg));
    register(
        "Example Waypoint (Type 3)",
        WaypointGroup.PLACEHOLDER_3,
        new Pose2d(2.0, 5.0, Rotation2d.kCW_90deg));
  }

  public static Waypoint register(String name, WaypointGroup group, Pose2d bluePose) {
    Waypoint waypoint = new Waypoint(name, group, bluePose);
    REGISTRY.put(name, waypoint);
    return waypoint;
  }

  public static List<Waypoint> all() {
    return Collections.unmodifiableList(new ArrayList<>(REGISTRY.values()));
  }

  public static List<Waypoint> inGroup(WaypointGroup group) {
    return REGISTRY.values().stream().filter(waypoint -> waypoint.group() == group).toList();
  }

  public static Optional<Waypoint> byName(String name) {
    return Optional.ofNullable(REGISTRY.get(name));
  }

  public static Optional<Waypoint> nearest(Pose2d robotPose) {
    return nearestOf(all(), robotPose);
  }

  public static Optional<Waypoint> nearestInGroup(Pose2d robotPose, WaypointGroup group) {
    return nearestOf(inGroup(group), robotPose);
  }

  private static Optional<Waypoint> nearestOf(List<Waypoint> candidates, Pose2d robotPose) {
    return candidates.stream()
        .min(Comparator.comparingDouble(waypoint -> waypoint.distanceFrom(robotPose)));
  }
}
