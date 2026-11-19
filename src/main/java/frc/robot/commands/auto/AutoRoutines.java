package frc.robot.commands.auto;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.FollowPathCommand;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.commands.PathfindingCommand;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.pathfinding.Pathfinding;
import com.pathplanner.lib.util.FlippingUtil;
import com.pathplanner.lib.util.PathPlannerLogging;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants;
import frc.robot.RobotState;
import frc.robot.commands.drive.DriveCommands;
import frc.robot.field.AllianceFlipUtil;
import frc.robot.field.FieldConstants;
import frc.robot.subsystems.drive.Drive;
import frc.robot.util.LocalADStarAK;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

public final class AutoRoutines {

  public static final PPHolonomicDriveController PATH_CONTROLLER =
      new PPHolonomicDriveController(
          new PIDConstants(
              Constants.Auto.TRANSLATION_P,
              Constants.Auto.TRANSLATION_I,
              Constants.Auto.TRANSLATION_D),
          new PIDConstants(
              Constants.Auto.ROTATION_P, Constants.Auto.ROTATION_I, Constants.Auto.ROTATION_D),
          Constants.LOOP_PERIOD_SECONDS);

  private AutoRoutines() {}

  public static void configure(Drive drive) {
    FlippingUtil.fieldSizeX = FieldConstants.FIELD_LENGTH;
    FlippingUtil.fieldSizeY = FieldConstants.FIELD_WIDTH;
    AutoBuilder.configureCustom(
        path -> new ChainedFollowPath(path, drive),
        () -> RobotState.getInstance().getPose(),
        drive::setPose,
        AllianceFlipUtil::shouldFlip,
        true);
    Pathfinding.setPathfinder(new LocalADStarAK());
    PathPlannerLogging.setLogActivePathCallback(
        poses -> Logger.recordOutput("PathPlanner/activePath", poses.toArray(new Pose2d[0])));
    PathPlannerLogging.setLogTargetPoseCallback(
        targetPose -> {
          Logger.recordOutput("PathPlanner/targetPose", targetPose);
          Logger.recordOutput(
              "PathPlanner/inaccuracy",
              targetPose
                  .getTranslation()
                  .getDistance(RobotState.getInstance().getPose().getTranslation()));
        });
  }

  public static LoggedDashboardChooser<Command> buildChooser(Drive drive) {
    LoggedDashboardChooser<Command> chooser = new LoggedDashboardChooser<>("Auto Chooser");
    chooser.addDefaultOption(Constants.Auto.DEFAULT_AUTO_NAME, Commands.none());
    for (String autoName : AutoBuilder.getAllAutoNames()) {
      chooser.addOption(autoName, new PathPlannerAuto(autoName));
    }
    chooser.addOption(
        "Drive Simple FF Characterization", DriveCommands.feedforwardCharacterization(drive));
    chooser.addOption(
        "Drive Wheel Radius Characterization", DriveCommands.wheelRadiusCharacterization(drive));
    chooser.addOption(
        "Drive SysId (Quasistatic Forward)",
        drive.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
    chooser.addOption(
        "Drive SysId (Quasistatic Reverse)",
        drive.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
    chooser.addOption(
        "Drive SysId (Dynamic Forward)", drive.sysIdDynamic(SysIdRoutine.Direction.kForward));
    chooser.addOption(
        "Drive SysId (Dynamic Reverse)", drive.sysIdDynamic(SysIdRoutine.Direction.kReverse));
    return chooser;
  }

  public static Command warmupCommand() {
    return FollowPathCommand.warmupCommand()
        .andThen(PathfindingCommand.warmupCommand())
        .ignoringDisable(true)
        .withName("PathPlannerWarmup");
  }
}
