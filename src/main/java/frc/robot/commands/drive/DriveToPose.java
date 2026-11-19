package frc.robot.commands.drive;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.DegreesPerSecondPerSecond;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.MetersPerSecondPerSecond;
import static edu.wpi.first.units.Units.Seconds;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Constants.Alignment;
import frc.robot.RobotState;
import frc.robot.subsystems.drive.Drive;
import frc.robot.util.LoggedTunableNumber;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

public class DriveToPose extends Command {

  private static final LoggedTunableNumber TRANSLATION_KP =
      new LoggedTunableNumber("DriveToPose/TranslationKp", Alignment.TRANSLATION_KP);

  private static final LoggedTunableNumber TRANSLATION_KD =
      new LoggedTunableNumber("DriveToPose/TranslationKd", Alignment.TRANSLATION_KD);

  private static final LoggedTunableNumber TRANSLATION_MAX_VELOCITY =
      new LoggedTunableNumber(
          "DriveToPose/TranslationMaxVelocity",
          Alignment.TRANSLATION_MAX_VELOCITY.in(MetersPerSecond));

  private static final LoggedTunableNumber TRANSLATION_MAX_ACCELERATION =
      new LoggedTunableNumber(
          "DriveToPose/TranslationMaxAcceleration",
          Alignment.TRANSLATION_MAX_ACCELERATION.in(MetersPerSecondPerSecond));

  static final LoggedTunableNumber ROTATION_KP =
      new LoggedTunableNumber("DriveToPose/RotationKp", Alignment.ROTATION_KP);

  static final LoggedTunableNumber ROTATION_KD =
      new LoggedTunableNumber("DriveToPose/RotationKd", Alignment.ROTATION_KD);

  static final LoggedTunableNumber ROTATION_MAX_VELOCITY_DEG =
      new LoggedTunableNumber(
          "DriveToPose/RotationMaxVelocityDegPerSec",
          Alignment.ROTATION_MAX_VELOCITY.in(DegreesPerSecond));

  static final LoggedTunableNumber ROTATION_MAX_ACCELERATION_DEG =
      new LoggedTunableNumber(
          "DriveToPose/RotationMaxAccelerationDegPerSecSq",
          Alignment.ROTATION_MAX_ACCELERATION.in(DegreesPerSecondPerSecond));

  private static final LoggedTunableNumber FF_MIN_RADIUS =
      new LoggedTunableNumber("DriveToPose/FFMinRadius", Alignment.FF_MIN_RADIUS.in(Meters));

  private static final LoggedTunableNumber FF_MAX_RADIUS =
      new LoggedTunableNumber("DriveToPose/FFMaxRadius", Alignment.FF_MAX_RADIUS.in(Meters));

  private static final LoggedTunableNumber TRANSLATION_TOLERANCE =
      new LoggedTunableNumber(
          "DriveToPose/TranslationTolerance", Alignment.TRANSLATION_TOLERANCE.in(Meters));

  private static final LoggedTunableNumber ROTATION_TOLERANCE_DEG =
      new LoggedTunableNumber(
          "DriveToPose/RotationToleranceDegrees", Alignment.ROTATION_TOLERANCE.in(Degrees));

  private static final LoggedTunableNumber VELOCITY_TOLERANCE =
      new LoggedTunableNumber(
          "DriveToPose/VelocityTolerance", Alignment.VELOCITY_TOLERANCE.in(MetersPerSecond));

  private static final LoggedTunableNumber OMEGA_TOLERANCE_DEG =
      new LoggedTunableNumber(
          "DriveToPose/OmegaToleranceDegPerSec", Alignment.OMEGA_TOLERANCE.in(DegreesPerSecond));

  private static final LoggedTunableNumber SETTLE_TIME =
      new LoggedTunableNumber("DriveToPose/SettleTimeSeconds", Alignment.SETTLE_TIME.in(Seconds));

  private final Drive drive;

  private final Supplier<Pose2d> targetSupplier;

  private final boolean finishAtGoal;

  private final PIDController translationController =
      new PIDController(
          TRANSLATION_KP.get(), 0.0, TRANSLATION_KD.get(), Constants.LOOP_PERIOD_SECONDS);

  private final ProfiledPIDController rotationController = createRotationController();

  private final Timer timer = new Timer();

  private final Debouncer atGoalDebouncer =
      new Debouncer(SETTLE_TIME.get(), Debouncer.DebounceType.kRising);

  private TrapezoidProfile translationProfile = createTranslationProfile();

  private TrapezoidProfile.State translationSetpoint = new TrapezoidProfile.State();

  private boolean atGoal = false;

  public DriveToPose(Drive drive, Supplier<Pose2d> targetSupplier, boolean finishAtGoal) {
    this.drive = drive;
    this.targetSupplier = targetSupplier;
    this.finishAtGoal = finishAtGoal;
    addRequirements(drive);
  }

  static ProfiledPIDController createRotationController() {
    ProfiledPIDController controller =
        new ProfiledPIDController(
            ROTATION_KP.get(),
            0.0,
            ROTATION_KD.get(),
            new TrapezoidProfile.Constraints(
                Math.toRadians(ROTATION_MAX_VELOCITY_DEG.get()),
                Math.toRadians(ROTATION_MAX_ACCELERATION_DEG.get())),
            Constants.LOOP_PERIOD_SECONDS);
    controller.enableContinuousInput(-Math.PI, Math.PI);
    return controller;
  }

  static void updateRotationController(ProfiledPIDController controller, int id) {
    LoggedTunableNumber.ifChanged(
        id,
        values -> {
          controller.setPID(values[0], 0.0, values[1]);
          controller.setConstraints(
              new TrapezoidProfile.Constraints(
                  Math.toRadians(values[2]), Math.toRadians(values[3])));
        },
        ROTATION_KP,
        ROTATION_KD,
        ROTATION_MAX_VELOCITY_DEG,
        ROTATION_MAX_ACCELERATION_DEG);
  }

  private static TrapezoidProfile createTranslationProfile() {
    return new TrapezoidProfile(
        new TrapezoidProfile.Constraints(
            TRANSLATION_MAX_VELOCITY.get(), TRANSLATION_MAX_ACCELERATION.get()));
  }

  @Override
  public void initialize() {
    updateTunables();
    RobotState robotState = RobotState.getInstance();
    Pose2d currentPose = robotState.getPose();
    Pose2d targetPose = targetSupplier.get();
    ChassisSpeeds fieldVelocity = robotState.getFieldVelocity();

    Translation2d toTarget = targetPose.getTranslation().minus(currentPose.getTranslation());
    double distance = toTarget.getNorm();
    double closingSpeed =
        distance > 1e-6
            ? (fieldVelocity.vxMetersPerSecond * toTarget.getX()
                    + fieldVelocity.vyMetersPerSecond * toTarget.getY())
                / distance
            : 0.0;
    translationSetpoint = new TrapezoidProfile.State(distance, Math.min(0.0, -closingSpeed));
    translationController.reset();
    rotationController.reset(
        currentPose.getRotation().getRadians(), fieldVelocity.omegaRadiansPerSecond);
    atGoalDebouncer.calculate(false);
    atGoal = false;
    timer.restart();
  }

  @Override
  public void execute() {
    updateTunables();
    RobotState robotState = RobotState.getInstance();
    Pose2d currentPose = robotState.getPose();
    Pose2d targetPose = targetSupplier.get();
    ChassisSpeeds robotVelocity = robotState.getRobotVelocity();

    Translation2d targetToRobot = currentPose.getTranslation().minus(targetPose.getTranslation());
    double distance = targetToRobot.getNorm();
    Rotation2d approachDirection = distance > 1e-6 ? targetToRobot.getAngle() : Rotation2d.kZero;

    translationSetpoint =
        translationProfile.calculate(
            Constants.LOOP_PERIOD_SECONDS, translationSetpoint, new TrapezoidProfile.State());
    double ffMin = FF_MIN_RADIUS.get();
    double ffScaler =
        MathUtil.clamp((distance - ffMin) / Math.max(FF_MAX_RADIUS.get() - ffMin, 1e-6), 0.0, 1.0);
    double translationTolerance = TRANSLATION_TOLERANCE.get();
    double velocityScalar =
        translationController.calculate(distance, translationSetpoint.position)
            + translationSetpoint.velocity * ffScaler;
    if (distance < translationTolerance) {
      velocityScalar = 0.0;
    }
    Translation2d fieldTranslationVelocity = new Translation2d(velocityScalar, approachDirection);

    double rotationError = targetPose.getRotation().minus(currentPose.getRotation()).getRadians();
    double rotationTolerance = Math.toRadians(ROTATION_TOLERANCE_DEG.get());
    double omega =
        rotationController.calculate(
                currentPose.getRotation().getRadians(), targetPose.getRotation().getRadians())
            + rotationController.getSetpoint().velocity;
    if (Math.abs(rotationError) < rotationTolerance) {
      omega = 0.0;
    }

    drive.runVelocity(
        ChassisSpeeds.fromFieldRelativeSpeeds(
            fieldTranslationVelocity.getX(),
            fieldTranslationVelocity.getY(),
            omega,
            currentPose.getRotation()));

    double linearSpeed =
        Math.hypot(robotVelocity.vxMetersPerSecond, robotVelocity.vyMetersPerSecond);
    atGoal =
        atGoalDebouncer.calculate(
            distance < translationTolerance
                && Math.abs(rotationError) < rotationTolerance
                && linearSpeed < VELOCITY_TOLERANCE.get()
                && Math.abs(robotVelocity.omegaRadiansPerSecond)
                    < Math.toRadians(OMEGA_TOLERANCE_DEG.get()));

    Translation2d setpointTranslation =
        targetPose
            .getTranslation()
            .plus(new Translation2d(translationSetpoint.position, approachDirection));
    Logger.recordOutput("DriveToPose/Target", targetPose);
    Logger.recordOutput(
        "DriveToPose/Setpoint",
        new Pose2d(
            setpointTranslation,
            Rotation2d.fromRadians(rotationController.getSetpoint().position)));
    Logger.recordOutput("DriveToPose/DistanceError", distance);
    Logger.recordOutput("DriveToPose/DistanceSetpoint", translationSetpoint.position);
    Logger.recordOutput("DriveToPose/VelocitySetpoint", translationSetpoint.velocity);
    Logger.recordOutput("DriveToPose/RotationError", rotationError);
    Logger.recordOutput("DriveToPose/AtGoal", atGoal);
    Logger.recordOutput("DriveToPose/ElapsedSeconds", timer.get());
  }

  @Override
  public boolean isFinished() {
    return finishAtGoal && atGoal;
  }

  @Override
  public void end(boolean interrupted) {
    drive.stop();
    timer.stop();
    Logger.recordOutput("DriveToPose/ElapsedSeconds", timer.get());
    Logger.recordOutput("DriveToPose/AtGoal", atGoal);
  }

  public boolean atGoal() {
    return atGoal;
  }

  private void updateTunables() {
    int id = hashCode();
    LoggedTunableNumber.ifChanged(
        id,
        values -> translationController.setPID(values[0], 0.0, values[1]),
        TRANSLATION_KP,
        TRANSLATION_KD);
    LoggedTunableNumber.ifChanged(
        id,
        values -> translationProfile = createTranslationProfile(),
        TRANSLATION_MAX_VELOCITY,
        TRANSLATION_MAX_ACCELERATION);
    LoggedTunableNumber.ifChanged(
        id, values -> atGoalDebouncer.setDebounceTime(values[0]), SETTLE_TIME);
    updateRotationController(rotationController, id);
  }
}
