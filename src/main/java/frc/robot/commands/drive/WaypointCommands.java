package frc.robot.commands.drive;

import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants;
import frc.robot.RobotState;
import frc.robot.field.FieldConstants;
import frc.robot.field.Waypoint;
import frc.robot.field.WaypointGroup;
import frc.robot.field.Waypoints;
import frc.robot.subsystems.drive.Drive;
import java.util.Optional;
import java.util.Set;
import java.util.function.DoubleSupplier;
import java.util.function.Function;
import java.util.function.Supplier;

public final class WaypointCommands {

  private WaypointCommands() {}

  public static Supplier<Optional<Pose2d>> fixed(Waypoint waypoint) {
    return () -> Optional.of(waypoint.pose());
  }

  public static Supplier<Optional<Pose2d>> fixed(Waypoint waypoint, Distance approachOffset) {
    return () -> Optional.of(waypoint.approach(approachOffset));
  }

  public static Supplier<Optional<Pose2d>> nearest() {
    return () -> Waypoints.nearest(RobotState.getInstance().getPose()).map(Waypoint::pose);
  }

  public static Supplier<Optional<Pose2d>> nearest(WaypointGroup group) {
    return () -> nearestWaypoint(group).map(Waypoint::pose);
  }

  public static Command alignTo(Drive drive, Supplier<Optional<Pose2d>> target) {
    return resolveOnce(drive, target, pose -> new DriveToPose(drive, () -> pose, true))
        .withTimeout(Constants.Alignment.TIMEOUT)
        .withName("AlignTo");
  }

  public static Command alignTo(Drive drive, Waypoint waypoint) {
    return alignTo(drive, fixed(waypoint));
  }

  public static Command alignToNearest(Drive drive, WaypointGroup group) {
    return alignTo(drive, nearest(group));
  }

  public static Command holdAlignTo(Drive drive, Supplier<Optional<Pose2d>> target) {
    return resolveOnce(drive, target, pose -> new DriveToPose(drive, () -> pose, false))
        .withName("HoldAlignTo");
  }

  public static Command holdAlignToNearest(Drive drive, WaypointGroup group) {
    return holdAlignTo(drive, nearest(group));
  }

  public static Command aimAt(
      Drive drive, DoubleSupplier forward, DoubleSupplier left, Supplier<Optional<Pose2d>> target) {
    return new AimAtWaypoint(drive, forward, left, target);
  }

  public static Command aimAtNearest(
      Drive drive, DoubleSupplier forward, DoubleSupplier left, WaypointGroup group) {
    return aimAt(drive, forward, left, nearest(group));
  }

  public static Command pathTo(Drive drive, Supplier<Optional<Pose2d>> target) {
    return new PathToWaypoint(drive, target);
  }

  public static Command pathThenAlign(Drive drive, Waypoint waypoint) {
    return pathThenAlign(drive, waypoint, FieldConstants.DEFAULT_APPROACH_OFFSET);
  }

  public static Command pathThenAlign(Drive drive, Waypoint waypoint, Distance approachOffset) {
    return Commands.sequence(
            pathTo(drive, fixed(waypoint, approachOffset)), alignTo(drive, fixed(waypoint)))
        .withName("PathThenAlign(" + waypoint.name() + ")");
  }

  public static Command pathThenAlignToNearest(Drive drive, WaypointGroup group) {
    return Commands.defer(
            () ->
                nearestWaypoint(group)
                    .map(waypoint -> pathThenAlign(drive, waypoint))
                    .orElseGet(Commands::none),
            Set.of(drive))
        .withName("PathThenAlignToNearest(" + group.name() + ")");
  }

  public static void registerNamedCommands(Drive drive) {
    for (Waypoint waypoint : Waypoints.all()) {
      NamedCommands.registerCommand("Align " + waypoint.name(), alignTo(drive, waypoint));
      NamedCommands.registerCommand("Path " + waypoint.name(), pathThenAlign(drive, waypoint));
    }
    for (WaypointGroup group : WaypointGroup.values()) {
      NamedCommands.registerCommand("Align Nearest " + group.name(), alignToNearest(drive, group));
      NamedCommands.registerCommand(
          "Path Nearest " + group.name(), pathThenAlignToNearest(drive, group));
    }
  }

  private static Optional<Waypoint> nearestWaypoint(WaypointGroup group) {
    return Waypoints.nearestInGroup(RobotState.getInstance().getPose(), group);
  }

  private static Command resolveOnce(
      Drive drive, Supplier<Optional<Pose2d>> target, Function<Pose2d, Command> factory) {
    return Commands.defer(() -> target.get().map(factory).orElseGet(Commands::none), Set.of(drive));
  }
}
