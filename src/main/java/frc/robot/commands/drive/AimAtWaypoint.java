package frc.robot.commands.drive;

import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.Seconds;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.Alignment;
import frc.robot.Constants.Operator;
import frc.robot.RobotState;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.DriveConstants;
import java.util.Optional;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

public class AimAtWaypoint extends Command {

  private final Drive drive;

  private final DoubleSupplier forward;

  private final DoubleSupplier left;

  private final Supplier<Optional<Pose2d>> targetSupplier;

  private final ProfiledPIDController rotationController = DriveToPose.createRotationController();

  private Optional<Pose2d> target = Optional.empty();

  private double aimError = Double.POSITIVE_INFINITY;

  public AimAtWaypoint(
      Drive drive,
      DoubleSupplier forward,
      DoubleSupplier left,
      Supplier<Optional<Pose2d>> targetSupplier) {
    this.drive = drive;
    this.forward = forward;
    this.left = left;
    this.targetSupplier = targetSupplier;
    addRequirements(drive);
  }

  @Override
  public void initialize() {
    DriveToPose.updateRotationController(rotationController, hashCode());
    RobotState robotState = RobotState.getInstance();
    target = targetSupplier.get();
    aimError = Double.POSITIVE_INFINITY;
    rotationController.reset(
        robotState.getRotation().getRadians(), robotState.getFieldVelocity().omegaRadiansPerSecond);
    target.ifPresent(pose -> Logger.recordOutput("Alignment/AimTarget", pose));
    Logger.recordOutput("Alignment/AimActive", target.isPresent());
  }

  @Override
  public void execute() {
    DriveToPose.updateRotationController(rotationController, hashCode());
    RobotState robotState = RobotState.getInstance();
    Pose2d robotPose = robotState.getPose();
    ChassisSpeeds fieldVelocity = robotState.getFieldVelocity();

    Translation2d linearVelocity =
        DriveCommands.fieldTranslationVelocity(
            forward.getAsDouble(),
            left.getAsDouble(),
            DriveConstants.MAX_LINEAR_SPEED
                * Operator.TRANSLATION_SCALE
                * Alignment.AIM_TRANSLATION_SCALE);

    double omega = 0.0;
    if (target.isPresent()) {
      double lookahead = Alignment.AIM_LOOKAHEAD.in(Seconds);
      Translation2d predictedPosition =
          robotPose
              .getTranslation()
              .plus(
                  new Translation2d(
                      fieldVelocity.vxMetersPerSecond * lookahead,
                      fieldVelocity.vyMetersPerSecond * lookahead));
      Translation2d toTarget = target.get().getTranslation().minus(predictedPosition);
      if (toTarget.getNorm() > 1e-3) {
        Rotation2d aimHeading = toTarget.getAngle();
        omega =
            rotationController.calculate(
                    robotPose.getRotation().getRadians(), aimHeading.getRadians())
                + rotationController.getSetpoint().velocity;
        aimError = Math.abs(aimHeading.minus(robotPose.getRotation()).getRadians());
      }
    }

    drive.runVelocity(
        ChassisSpeeds.fromFieldRelativeSpeeds(
            linearVelocity.getX(), linearVelocity.getY(), omega, robotPose.getRotation()));

    Logger.recordOutput("Alignment/AimError", aimError);
    Logger.recordOutput("Alignment/AimLocked", isLocked());
  }

  @Override
  public void end(boolean interrupted) {
    drive.stop();
    aimError = Double.POSITIVE_INFINITY;
    Logger.recordOutput("Alignment/AimActive", false);
    Logger.recordOutput("Alignment/AimLocked", false);
  }

  public boolean isLocked() {
    return aimError < Alignment.AIM_LOCK_TOLERANCE.in(Radians);
  }
}
