package frc.robot.commands.auto;

import static edu.wpi.first.units.Units.Meters;

import com.pathplanner.lib.commands.FollowPathCommand;
import com.pathplanner.lib.path.IdealStartingState;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.Waypoint;
import com.pathplanner.lib.util.FlippingUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.RobotState;
import frc.robot.field.AllianceFlipUtil;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.DriveConstants;
import java.util.ArrayList;
import java.util.List;
import org.littletonrobotics.junction.Logger;

public class ChainedFollowPath extends Command {

  private final PathPlannerPath originalPath;

  private final Drive drive;

  private Command followCommand;

  public ChainedFollowPath(PathPlannerPath path, Drive drive) {
    this.originalPath = path;
    this.drive = drive;
    followCommand = createFollowCommand(path);
    addRequirements(followCommand.getRequirements());
    setName("ChainedFollowPath(" + path.name + ")");
  }

  public static PathPlannerPath reanchor(
      PathPlannerPath path, Pose2d bluePose, double linearSpeedMetersPerSec) {
    List<Waypoint> waypoints = new ArrayList<>(path.getWaypoints());
    Waypoint first = waypoints.get(0);
    Translation2d offset = bluePose.getTranslation().minus(first.anchor());
    Translation2d nextControl =
        first.nextControl() == null ? null : first.nextControl().plus(offset);
    waypoints.set(0, new Waypoint(null, bluePose.getTranslation(), nextControl));

    PathPlannerPath reanchored =
        new PathPlannerPath(
            waypoints,
            path.getRotationTargets(),
            path.getPointTowardsZones(),
            path.getConstraintZones(),
            path.getEventMarkers(),
            path.getGlobalConstraints(),
            new IdealStartingState(linearSpeedMetersPerSec, bluePose.getRotation()),
            path.getGoalEndState(),
            path.isReversed());
    reanchored.name = path.name;
    reanchored.preventFlipping = path.preventFlipping;
    return reanchored;
  }

  @Override
  public void initialize() {
    RobotState robotState = RobotState.getInstance();
    Pose2d robotPose = robotState.getPose();
    ChassisSpeeds robotVelocity = robotState.getRobotVelocity();
    boolean flipped = AllianceFlipUtil.shouldFlip() && !originalPath.preventFlipping;
    Pose2d bluePose = flipped ? FlippingUtil.flipFieldPose(robotPose) : robotPose;

    PathPlannerPath path = originalPath;
    double reanchorDistance = Double.NaN;
    if (!originalPath.isChoreoPath() && originalPath.getWaypoints().size() >= 2) {
      reanchorDistance =
          bluePose.getTranslation().getDistance(originalPath.getWaypoints().get(0).anchor());
      if (reanchorDistance >= Constants.Auto.CHAIN_MIN_REANCHOR_DISTANCE.in(Meters)
          && reanchorDistance <= Constants.Auto.CHAIN_MAX_REANCHOR_DISTANCE.in(Meters)) {
        path =
            reanchor(
                originalPath,
                bluePose,
                Math.hypot(robotVelocity.vxMetersPerSecond, robotVelocity.vyMetersPerSecond));
      }
    }
    Logger.recordOutput("Auto/Chain/ReanchorDistance", reanchorDistance);
    Logger.recordOutput("Auto/Chain/Reanchored", path != originalPath);

    followCommand = createFollowCommand(path);
    followCommand.initialize();
  }

  @Override
  public void execute() {
    followCommand.execute();
  }

  @Override
  public boolean isFinished() {
    return followCommand.isFinished();
  }

  @Override
  public void end(boolean interrupted) {
    followCommand.end(interrupted);
  }

  private Command createFollowCommand(PathPlannerPath path) {
    RobotState robotState = RobotState.getInstance();
    return new FollowPathCommand(
        path,
        robotState::getPose,
        robotState::getRobotVelocity,
        drive::runPathVelocity,
        AutoRoutines.PATH_CONTROLLER,
        DriveConstants.PATHPLANNER_CONFIG,
        AllianceFlipUtil::shouldFlip,
        drive);
  }
}
