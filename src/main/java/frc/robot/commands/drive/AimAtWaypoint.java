package frc.robot.commands.drive;

import static edu.wpi.first.units.Units.Radians;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import java.util.Optional;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;
import swervelib.SwerveInputStream;

public class AimAtWaypoint extends Command {

  private final SwerveSubsystem swerve;

  private final Supplier<Optional<Pose2d>> targetSupplier;

  private final SwerveInputStream inputStream;

  private Pose2d target;

  public AimAtWaypoint(
      SwerveSubsystem swerve,
      SwerveInputStream driverStream,
      Supplier<Optional<Pose2d>> targetSupplier) {
    this.swerve = swerve;
    this.targetSupplier = targetSupplier;
    this.target = swerve.getPose();
    this.inputStream = driverStream.copy();
    addRequirements(swerve);
  }

  @Override
  public void initialize() {
    target = targetSupplier.get().orElse(swerve.getPose());
    inputStream
        .aim(() -> target)
        .aimWhile(true)
        .aimLookahead(Constants.Alignment.AIM_LOOKAHEAD)
        .scaleTranslation(Constants.Alignment.AIM_TRANSLATION_SCALE);
    Logger.recordOutput("Alignment/AimTarget", target);
    Logger.recordOutput("Alignment/AimActive", true);
  }

  @Override
  public void execute() {
    swerve.driveFieldOriented(inputStream.get());
    Logger.recordOutput("Alignment/AimError", aimError());
    Logger.recordOutput("Alignment/AimLocked", isLocked());
  }

  @Override
  public boolean isFinished() {
    return false;
  }

  @Override
  public void end(boolean interrupted) {
    inputStream.aimWhile(false).scaleTranslation(Constants.Operator.TRANSLATION_SCALE);
    Logger.recordOutput("Alignment/AimActive", false);
    Logger.recordOutput("Alignment/AimLocked", false);
  }

  public boolean isLocked() {
    return aimError() <= Constants.Alignment.AIM_LOCK_TOLERANCE.in(Radians);
  }

  private double aimError() {
    Pose2d robotPose = swerve.getPose();
    return Math.abs(
        target
            .getTranslation()
            .minus(robotPose.getTranslation())
            .getAngle()
            .minus(robotPose.getRotation())
            .getRadians());
  }
}
