package frc.robot.commands.drive;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
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

  private final Supplier<Optional<Pose2d>> targetSupplier;

  private Command pathCommand = Commands.none();

  public PathToWaypoint(SwerveSubsystem swerve, Supplier<Optional<Pose2d>> targetSupplier) {
    this.targetSupplier = targetSupplier;
    addRequirements(swerve);
  }

  @Override
  public void initialize() {
    Optional<Pose2d> target = targetSupplier.get();
    if (target.isEmpty()) {
      pathCommand = Commands.none();
    } else {
      Logger.recordOutput("Alignment/PathTarget", target.get());
      pathCommand =
          AutoBuilder.pathfindToPose(
              target.get(), CONSTRAINTS, Constants.Auto.PATHFIND_GOAL_END_VELOCITY);
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
