package frc.robot.commands.drive;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Radians;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import java.util.Optional;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;
import swervelib.SwerveInputStream;

public class SnapToWaypoint extends Command {

  private final SwerveSubsystem swerve;

  private final Supplier<Optional<Pose2d>> targetSupplier;

  private final boolean endAtTarget;

  private final SwerveInputStream inputStream;

  private Pose2d target;

  public SnapToWaypoint(SwerveSubsystem swerve, Supplier<Optional<Pose2d>> targetSupplier) {
    this(swerve, targetSupplier, true);
  }

  public SnapToWaypoint(
      SwerveSubsystem swerve, Supplier<Optional<Pose2d>> targetSupplier, boolean endAtTarget) {
    this.swerve = swerve;
    this.targetSupplier = targetSupplier;
    this.endAtTarget = endAtTarget;
    this.target = swerve.getPose();
    this.inputStream =
        swerve
            .zeroInputStream()
            .driveToPose(
                () -> target,
                Constants.Alignment.translationController(),
                Constants.Alignment.rotationController())
            .driveToPoseEnabled(true);
    addRequirements(swerve);
  }

  @Override
  public void initialize() {
    target = targetSupplier.get().orElse(swerve.getPose());
    Logger.recordOutput("Alignment/SnapTarget", target);
    Logger.recordOutput("Alignment/SnapActive", true);
  }

  @Override
  public void execute() {
    swerve.driveFieldOriented(inputStream.get());
    Logger.recordOutput("Alignment/SnapTranslationError", translationError());
    Logger.recordOutput("Alignment/SnapRotationError", rotationError());
  }

  @Override
  public boolean isFinished() {
    return endAtTarget && atTarget();
  }

  @Override
  public void end(boolean interrupted) {
    swerve.driveFieldOriented(new edu.wpi.first.math.kinematics.ChassisSpeeds());
    Logger.recordOutput("Alignment/SnapActive", false);
  }

  public boolean atTarget() {
    return inputStream.atTargetPose(Constants.Alignment.SNAP_TRANSLATION_TOLERANCE.in(Meters))
        && rotationError() <= Constants.Alignment.SNAP_ROTATION_TOLERANCE.in(Radians);
  }

  private double translationError() {
    return swerve.getPose().getTranslation().getDistance(target.getTranslation());
  }

  private double rotationError() {
    return Math.abs(target.getRotation().minus(swerve.getPose().getRotation()).getRadians());
  }
}
