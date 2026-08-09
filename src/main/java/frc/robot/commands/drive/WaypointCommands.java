package frc.robot.commands.drive;

import static edu.wpi.first.units.Units.Meters;

import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants;
import frc.robot.field.FieldConstants;
import frc.robot.field.Waypoint;
import frc.robot.field.WaypointGroup;
import frc.robot.field.Waypoints;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import java.util.Optional;
import java.util.function.Supplier;
import swervelib.SwerveInputStream;

public final class WaypointCommands {

  private WaypointCommands() {}

  public static Supplier<Optional<Pose2d>> fixed(Waypoint waypoint) {
    return () -> Optional.of(waypoint.pose());
  }

  public static Supplier<Optional<Pose2d>> fixed(Waypoint waypoint, Distance approachOffset) {
    return () -> Optional.of(waypoint.approach(approachOffset));
  }

  public static Supplier<Optional<Pose2d>> nearest(SwerveSubsystem swerve) {
    return () -> Waypoints.nearest(swerve.getPose()).map(Waypoint::pose);
  }

  public static Supplier<Optional<Pose2d>> nearest(SwerveSubsystem swerve, WaypointGroup group) {
    return () -> Waypoints.nearestInGroup(swerve.getPose(), group).map(Waypoint::pose);
  }

  public static Command snapTo(SwerveSubsystem swerve, Supplier<Optional<Pose2d>> target) {
    return new SnapToWaypoint(swerve, target, true).withTimeout(Constants.Alignment.SNAP_TIMEOUT);
  }

  public static Command snapTo(SwerveSubsystem swerve, Waypoint waypoint) {
    return snapTo(swerve, fixed(waypoint));
  }

  public static Command snapToNearest(SwerveSubsystem swerve, WaypointGroup group) {
    return snapTo(swerve, nearest(swerve, group));
  }

  public static Command holdSnapTo(SwerveSubsystem swerve, Supplier<Optional<Pose2d>> target) {
    return new SnapToWaypoint(swerve, target, false);
  }

  public static Command holdSnapToNearest(SwerveSubsystem swerve, WaypointGroup group) {
    return holdSnapTo(swerve, nearest(swerve, group));
  }

  public static Command aimAt(
      SwerveSubsystem swerve, SwerveInputStream driverStream, Supplier<Optional<Pose2d>> target) {
    return new AimAtWaypoint(swerve, driverStream, target);
  }

  public static Command aimAtNearest(
      SwerveSubsystem swerve, SwerveInputStream driverStream, WaypointGroup group) {
    return aimAt(swerve, driverStream, nearest(swerve, group));
  }

  public static Command pathTo(SwerveSubsystem swerve, Supplier<Optional<Pose2d>> target) {
    return new PathToWaypoint(swerve, target);
  }

  public static Command pathThenSnap(SwerveSubsystem swerve, Waypoint waypoint) {
    return pathThenSnap(swerve, waypoint, FieldConstants.DEFAULT_APPROACH_OFFSET);
  }

  public static Command pathThenSnap(
      SwerveSubsystem swerve, Waypoint waypoint, Distance approachOffset) {
    return Commands.sequence(
        pathTo(swerve, fixed(waypoint, approachOffset)), snapTo(swerve, fixed(waypoint)));
  }

  public static Command pathThenSnapToNearest(SwerveSubsystem swerve, WaypointGroup group) {
    return Commands.sequence(
        pathTo(
            swerve,
            () ->
                Waypoints.nearestInGroup(swerve.getPose(), group)
                    .map(
                        waypoint ->
                            waypoint.approach(FieldConstants.DEFAULT_APPROACH_OFFSET.in(Meters)))),
        snapToNearest(swerve, group));
  }

  public static void registerNamedCommands(SwerveSubsystem swerve) {
    for (Waypoint waypoint : Waypoints.all()) {
      NamedCommands.registerCommand("Snap " + waypoint.name(), snapTo(swerve, waypoint));
      NamedCommands.registerCommand("Path " + waypoint.name(), pathThenSnap(swerve, waypoint));
    }
    for (WaypointGroup group : WaypointGroup.values()) {
      NamedCommands.registerCommand("Snap Nearest " + group.name(), snapToNearest(swerve, group));
      NamedCommands.registerCommand(
          "Path Nearest " + group.name(), pathThenSnapToNearest(swerve, group));
    }
  }
}
