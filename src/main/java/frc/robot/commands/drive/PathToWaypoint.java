package frc.robot.commands.drive;

import com.pathplanner.lib.commands.PathfindingCommand;
import com.pathplanner.lib.path.PathConstraints;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants;
import frc.robot.RobotState;
import frc.robot.commands.auto.AutoRoutines;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.DriveConstants;
import java.util.Optional;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

public class PathToWaypoint extends Command {

  public static final PathConstraints CONSTRAINTS =
      new PathConstraints(
          Constants.Auto.PATHFIND_MAX_VELOCITY,
          Constants.Auto.PATHFIND_MAX_ACCELERATION,
          Constants.Auto.PATHFIND_MAX_ANGULAR_VELOCITY,
          Constants.Auto.PATHFIND_MAX_ANGULAR_ACCELERATION);

  private final Drive drive;

  private final Supplier<Optional<Pose2d>> targetSupplier;

  private Command pathCommand = Commands.none();

  public PathToWaypoint(Drive drive, Supplier<Optional<Pose2d>> targetSupplier) {
    this.drive = drive;
    this.targetSupplier = targetSupplier;
    addRequirements(drive);
  }

  @Override
  public void initialize() {
    Optional<Pose2d> target = targetSupplier.get();
    if (target.isEmpty()) {
      pathCommand = Commands.none();
    } else {
      Logger.recordOutput("Alignment/PathTarget", target.get());
      RobotState robotState = RobotState.getInstance();
      pathCommand =
          new PathfindingCommand(
              target.get(),
              CONSTRAINTS,
              Constants.Auto.PATHFIND_GOAL_END_VELOCITY,
              robotState::getPose,
              robotState::getRobotVelocity,
              drive::runPathVelocity,
              AutoRoutines.PATH_CONTROLLER,
              DriveConstants.PATHPLANNER_CONFIG);
    }
    pathCommand.initialize();
  }

  @Override
  public void execute() {
    pathCommand.execute();
  }

  @Override
  public boolean isFinished() {
    return pathCommand.isFinished();
  }

  @Override
  public void end(boolean interrupted) {
    pathCommand.end(interrupted);
  }
}
