package frc.robot.subsystems.swervedrive;

import static edu.wpi.first.units.Units.MetersPerSecond;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.pathfinding.LocalADStar;
import com.pathplanner.lib.pathfinding.Pathfinding;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants;
import frc.robot.field.AllianceFlipUtil;
import java.io.File;
import java.util.Optional;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;
import swervelib.SwerveDrive;
import swervelib.SwerveDriveTest;
import swervelib.SwerveInputStream;
import swervelib.parser.SwerveParser;
import swervelib.telemetry.SwerveDriveTelemetry;
import swervelib.telemetry.SwerveDriveTelemetry.TelemetryVerbosity;

public class SwerveSubsystem extends SubsystemBase {

  private static final double SYSID_MAX_VOLTS = 12.0;

  private static final double SYSID_DELAY_SECONDS = 3.0;

  private static final double SYSID_QUASISTATIC_TIMEOUT_SECONDS = 5.0;

  private static final double SYSID_DYNAMIC_TIMEOUT_SECONDS = 3.0;

  private final SwerveDrive swerveDrive;

  private final Alert pathPlannerAlert =
      new Alert("Failed to configure PathPlanner.", AlertType.kError);

  public SwerveSubsystem(File directory) {
    SwerveDriveTelemetry.verbosity = TelemetryVerbosity.HIGH;
    try {
      swerveDrive =
          new SwerveParser(directory)
              .createSwerveDrive(
                  Constants.Drive.MAX_SPEED.in(MetersPerSecond), Constants.Drive.STARTING_POSE);
    } catch (Exception e) {
      throw new RuntimeException(e);
    }
    swerveDrive.setHeadingCorrection(Constants.Drive.HEADING_CORRECTION);
    swerveDrive.setCosineCompensator(Constants.Drive.COSINE_COMPENSATION);
    swerveDrive.setAngularVelocityCompensation(
        Constants.Drive.ANGULAR_VELOCITY_COMPENSATION_TELEOP,
        Constants.Drive.ANGULAR_VELOCITY_COMPENSATION_AUTO,
        Constants.Drive.ANGULAR_VELOCITY_COEFFICIENT);
    swerveDrive.setChassisDiscretization(
        Constants.Drive.CHASSIS_DISCRETIZATION, Constants.Drive.CHASSIS_DISCRETIZATION_SECONDS);
    swerveDrive.setModuleEncoderAutoSynchronize(
        Constants.Drive.MODULE_ENCODER_AUTO_SYNCHRONIZE,
        Constants.Drive.MODULE_ENCODER_AUTO_SYNCHRONIZE_DEADBAND);
    configurePathPlanner();
  }

  public SwerveDrive getSwerveDrive() {
    return swerveDrive;
  }

  public Pose2d getPose() {
    return swerveDrive.getPose();
  }

  public void resetOdometry(Pose2d pose) {
    swerveDrive.resetOdometry(pose);
  }

  public Rotation2d getHeading() {
    return swerveDrive.getOdometryHeading();
  }

  public ChassisSpeeds getFieldVelocity() {
    return swerveDrive.getFieldVelocity();
  }

  public ChassisSpeeds getRobotVelocity() {
    return swerveDrive.getRobotVelocity();
  }

  public void setChassisSpeeds(ChassisSpeeds speeds) {
    swerveDrive.setChassisSpeeds(speeds);
  }

  public void driveFieldOriented(ChassisSpeeds velocity) {
    Logger.recordOutput("Swerve/CommandedSpeeds", velocity);
    Logger.recordOutput("Swerve/CommandedOmega", velocity.omegaRadiansPerSecond);
    swerveDrive.driveFieldOriented(velocity);
  }

  public Command driveFieldOriented(Supplier<ChassisSpeeds> velocity) {
    return run(() -> swerveDrive.driveFieldOriented(velocity.get()));
  }

  public void zeroGyro() {
    swerveDrive.zeroGyro();
  }

  public void zeroGyroWithAlliance() {
    zeroGyro();
    if (AllianceFlipUtil.shouldFlip()) {
      resetOdometry(new Pose2d(getPose().getTranslation(), Rotation2d.k180deg));
    } else {
      resetOdometry(new Pose2d(getPose().getTranslation(), Rotation2d.kZero));
    }
  }

  public void setMotorBrake(boolean brake) {
    swerveDrive.setMotorIdleMode(brake);
  }

  public void lock() {
    swerveDrive.lockPose();
  }

  public Optional<Pose2d> getSimulationPose() {
    return swerveDrive.getSimulationDriveTrainPose();
  }

  public SwerveInputStream zeroInputStream() {
    return SwerveInputStream.of(swerveDrive, () -> 0.0, () -> 0.0)
        .withControllerRotationAxis(() -> 0.0);
  }

  public void configurePathPlanner() {
    try {
      RobotConfig config = RobotConfig.fromGUISettings();
      AutoBuilder.configure(
          this::getPose,
          this::resetOdometry,
          this::getRobotVelocity,
          (speeds, feedforwards) ->
              swerveDrive.drive(
                  speeds,
                  swerveDrive.kinematics.toSwerveModuleStates(speeds),
                  feedforwards.linearForces()),
          new PPHolonomicDriveController(
              new PIDConstants(
                  Constants.Auto.TRANSLATION_P,
                  Constants.Auto.TRANSLATION_I,
                  Constants.Auto.TRANSLATION_D),
              new PIDConstants(
                  Constants.Auto.ROTATION_P, Constants.Auto.ROTATION_I, Constants.Auto.ROTATION_D)),
          config,
          AllianceFlipUtil::shouldFlip,
          this);
      pathPlannerAlert.set(false);
    } catch (Exception e) {
      DriverStation.reportError("Failed to configure PathPlanner: " + e.getMessage(), false);
      pathPlannerAlert.setText("Failed to configure PathPlanner: " + e.getMessage());
      pathPlannerAlert.set(true);
    }
    Pathfinding.setPathfinder(new LocalADStar());
  }

  public Command sysIdDriveMotorCommand() {
    return SwerveDriveTest.generateSysIdCommand(
        SwerveDriveTest.setDriveSysIdRoutine(
            new SysIdRoutine.Config(), this, swerveDrive, SYSID_MAX_VOLTS, true),
        SYSID_DELAY_SECONDS,
        SYSID_QUASISTATIC_TIMEOUT_SECONDS,
        SYSID_DYNAMIC_TIMEOUT_SECONDS);
  }

  public Command sysIdAngleMotorCommand() {
    return SwerveDriveTest.generateSysIdCommand(
        SwerveDriveTest.setAngleSysIdRoutine(new SysIdRoutine.Config(), this, swerveDrive),
        SYSID_DELAY_SECONDS,
        SYSID_QUASISTATIC_TIMEOUT_SECONDS,
        SYSID_DYNAMIC_TIMEOUT_SECONDS);
  }

  @Override
  public void periodic() {
    Logger.recordOutput("Swerve/Pose", getPose());
    Logger.recordOutput("Swerve/ModuleStates", swerveDrive.getStates());
    Logger.recordOutput("Swerve/ModulePositions", swerveDrive.getModulePositions());
    Logger.recordOutput("Swerve/FieldVelocity", getFieldVelocity());
    Logger.recordOutput("Swerve/RobotVelocity", getRobotVelocity());
    Logger.recordOutput("Swerve/Heading", getHeading());
    Optional<Pose2d> simulationPose = getSimulationPose();
    if (simulationPose.isPresent()) {
      Logger.recordOutput("Swerve/SimulationPose", simulationPose.get());
    }
  }
}
