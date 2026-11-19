package frc.robot.subsystems.drive;

import static edu.wpi.first.units.Units.Volts;

import com.pathplanner.lib.util.DriveFeedforwards;
import com.pathplanner.lib.util.swerve.SwerveSetpoint;
import com.pathplanner.lib.util.swerve.SwerveSetpointGenerator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants;
import frc.robot.RobotState;
import java.util.concurrent.locks.Lock;
import java.util.concurrent.locks.ReentrantLock;
import java.util.function.Consumer;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Drive extends SubsystemBase {

  static final Lock ODOMETRY_LOCK = new ReentrantLock();

  private final GyroIO gyroIO;

  private final GyroIOInputsAutoLogged gyroInputs = new GyroIOInputsAutoLogged();

  private final Module[] modules = new Module[DriveConstants.MODULES.length];

  private final Consumer<Pose2d> simulationPoseResetter;

  private final SysIdRoutine sysId;

  private final SwerveSetpointGenerator setpointGenerator =
      new SwerveSetpointGenerator(
          DriveConstants.PATHPLANNER_CONFIG, DriveConstants.MAX_STEER_VELOCITY);

  private final Alert gyroDisconnectedAlert =
      new Alert("Disconnected gyro, using kinematics as fallback.", AlertType.kError);

  private SwerveSetpoint previousSetpoint;

  private Rotation2d rawGyroRotation = Rotation2d.kZero;

  private SwerveModulePosition[] lastModulePositions = new SwerveModulePosition[modules.length];

  public Drive(
      GyroIO gyroIO,
      ModuleIO frontLeft,
      ModuleIO frontRight,
      ModuleIO backLeft,
      ModuleIO backRight,
      Consumer<Pose2d> simulationPoseResetter) {
    this.gyroIO = gyroIO;
    this.simulationPoseResetter = simulationPoseResetter;
    ModuleIO[] moduleIOs = {frontLeft, frontRight, backLeft, backRight};
    for (int i = 0; i < modules.length; i++) {
      modules[i] = new Module(moduleIOs[i], DriveConstants.MODULES[i].name());
      lastModulePositions[i] = new SwerveModulePosition();
    }
    previousSetpoint = measuredSetpoint();

    SparkOdometryThread.getInstance().start();

    sysId =
        new SysIdRoutine(
            new SysIdRoutine.Config(
                null,
                null,
                null,
                state -> Logger.recordOutput("Drive/SysIdState", state.toString())),
            new SysIdRoutine.Mechanism(
                voltage -> runCharacterization(voltage.in(Volts)), null, this));
  }

  @Override
  public void periodic() {
    ODOMETRY_LOCK.lock();
    try {
      gyroIO.updateInputs(gyroInputs);
      Logger.processInputs("Drive/Gyro", gyroInputs);
      for (Module module : modules) {
        module.periodic();
      }
    } finally {
      ODOMETRY_LOCK.unlock();
    }

    if (DriverStation.isDisabled()) {
      for (Module module : modules) {
        module.stop();
      }
      previousSetpoint = measuredSetpoint();
      Logger.recordOutput("Drive/ModuleStates/Setpoints", new SwerveModuleState[] {});
      Logger.recordOutput("Drive/ChassisSpeeds/Setpoint", new ChassisSpeeds());
    }

    updateOdometry();

    ChassisSpeeds measuredSpeeds = DriveConstants.KINEMATICS.toChassisSpeeds(getModuleStates());
    if (gyroInputs.connected) {
      measuredSpeeds.omegaRadiansPerSecond = gyroInputs.yawVelocityRadPerSec;
    }
    RobotState.getInstance().addVelocityObservation(measuredSpeeds);
    Logger.recordOutput("Drive/ChassisSpeeds/Measured", measuredSpeeds);

    gyroDisconnectedAlert.set(
        !gyroInputs.connected && Constants.CURRENT_MODE != Constants.RobotMode.SIM);
  }

  private void updateOdometry() {
    int sampleCount = modules[0].getOdometryTimestamps().length;
    for (Module module : modules) {
      sampleCount = Math.min(sampleCount, module.getOdometryPositions().length);
    }
    if (gyroInputs.connected) {
      sampleCount = Math.min(sampleCount, gyroInputs.odometryYawPositions.length);
    }
    double[] sampleTimestamps = modules[0].getOdometryTimestamps();
    for (int i = 0; i < sampleCount; i++) {
      SwerveModulePosition[] modulePositions = new SwerveModulePosition[modules.length];
      SwerveModulePosition[] moduleDeltas = new SwerveModulePosition[modules.length];
      for (int moduleIndex = 0; moduleIndex < modules.length; moduleIndex++) {
        modulePositions[moduleIndex] = modules[moduleIndex].getOdometryPositions()[i];
        moduleDeltas[moduleIndex] =
            new SwerveModulePosition(
                modulePositions[moduleIndex].distanceMeters
                    - lastModulePositions[moduleIndex].distanceMeters,
                modulePositions[moduleIndex].angle);
        lastModulePositions[moduleIndex] = modulePositions[moduleIndex];
      }
      if (gyroInputs.connected) {
        rawGyroRotation = gyroInputs.odometryYawPositions[i];
      } else {
        Twist2d twist = DriveConstants.KINEMATICS.toTwist2d(moduleDeltas);
        rawGyroRotation = rawGyroRotation.plus(new Rotation2d(twist.dtheta));
      }
      RobotState.getInstance()
          .addOdometryObservation(sampleTimestamps[i], rawGyroRotation, modulePositions);
    }
  }

  public void runVelocity(ChassisSpeeds robotRelativeSpeeds) {
    previousSetpoint =
        setpointGenerator.generateSetpoint(
            previousSetpoint, robotRelativeSpeeds, Constants.LOOP_PERIOD_SECONDS);
    SwerveModuleState[] states = previousSetpoint.moduleStates();
    double[] accelerations = previousSetpoint.feedforwards().accelerationsMPSSq();
    for (int i = 0; i < modules.length; i++) {
      modules[i].runSetpoint(states[i], accelerations[i]);
    }
    Logger.recordOutput("Drive/ChassisSpeeds/Desired", robotRelativeSpeeds);
    Logger.recordOutput("Drive/ChassisSpeeds/Setpoint", previousSetpoint.robotRelativeSpeeds());
    Logger.recordOutput("Drive/ModuleStates/Setpoints", states);
  }

  public void runPathVelocity(ChassisSpeeds robotRelativeSpeeds, DriveFeedforwards feedforwards) {
    ChassisSpeeds discreteSpeeds =
        ChassisSpeeds.discretize(robotRelativeSpeeds, Constants.LOOP_PERIOD_SECONDS);
    SwerveModuleState[] states = DriveConstants.KINEMATICS.toSwerveModuleStates(discreteSpeeds);
    SwerveDriveKinematics.desaturateWheelSpeeds(states, DriveConstants.MAX_LINEAR_SPEED);
    double[] accelerations = feedforwards.accelerationsMPSSq();
    for (int i = 0; i < modules.length; i++) {
      Rotation2d currentAngle = modules[i].getAngle();
      boolean flipped = Math.abs(states[i].angle.minus(currentAngle).getRadians()) > Math.PI / 2.0;
      states[i].optimize(currentAngle);
      double acceleration = i < accelerations.length ? accelerations[i] : 0.0;
      modules[i].runSetpoint(states[i], flipped ? -acceleration : acceleration);
    }
    previousSetpoint = new SwerveSetpoint(robotRelativeSpeeds, states, feedforwards);
    Logger.recordOutput("Drive/ChassisSpeeds/Desired", robotRelativeSpeeds);
    Logger.recordOutput("Drive/ChassisSpeeds/Setpoint", robotRelativeSpeeds);
    Logger.recordOutput("Drive/ModuleStates/Setpoints", states);
  }

  public void stop() {
    runVelocity(new ChassisSpeeds());
  }

  public void stopWithX() {
    SwerveModuleState[] states = new SwerveModuleState[modules.length];
    for (int i = 0; i < modules.length; i++) {
      states[i] = new SwerveModuleState(0.0, DriveConstants.MODULE_TRANSLATIONS[i].getAngle());
      modules[i].runSetpoint(states[i], 0.0);
    }
    previousSetpoint =
        new SwerveSetpoint(new ChassisSpeeds(), states, DriveFeedforwards.zeros(modules.length));
    Logger.recordOutput("Drive/ModuleStates/Setpoints", states);
  }

  public void runCharacterization(double volts) {
    for (Module module : modules) {
      module.runCharacterization(volts);
    }
    previousSetpoint = measuredSetpoint();
  }

  public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
    return run(() -> runCharacterization(0.0))
        .withTimeout(1.0)
        .andThen(sysId.quasistatic(direction));
  }

  public Command sysIdDynamic(SysIdRoutine.Direction direction) {
    return run(() -> runCharacterization(0.0)).withTimeout(1.0).andThen(sysId.dynamic(direction));
  }

  public double[] getWheelRadiusCharacterizationPositions() {
    double[] positions = new double[modules.length];
    for (int i = 0; i < modules.length; i++) {
      positions[i] = modules[i].getWheelRadiusCharacterizationPosition();
    }
    return positions;
  }

  public double getFFCharacterizationVelocity() {
    double velocity = 0.0;
    for (Module module : modules) {
      velocity += module.getFFCharacterizationVelocity() / modules.length;
    }
    return velocity;
  }

  public void setDriveBrakeMode(boolean enabled) {
    for (Module module : modules) {
      module.setDriveBrakeMode(enabled);
    }
  }

  public void setPose(Pose2d pose) {
    simulationPoseResetter.accept(pose);
    RobotState.getInstance().resetPose(pose);
  }

  public void resetHeading(Rotation2d heading) {
    RobotState.getInstance().resetPose(new Pose2d(getPose().getTranslation(), heading));
  }

  public Pose2d getPose() {
    return RobotState.getInstance().getPose();
  }

  public Rotation2d getRotation() {
    return RobotState.getInstance().getRotation();
  }

  public ChassisSpeeds getRobotVelocity() {
    return RobotState.getInstance().getRobotVelocity();
  }

  @AutoLogOutput(key = "Drive/ModuleStates/Measured")
  public SwerveModuleState[] getModuleStates() {
    SwerveModuleState[] states = new SwerveModuleState[modules.length];
    for (int i = 0; i < modules.length; i++) {
      states[i] = modules[i].getState();
    }
    return states;
  }

  public SwerveModulePosition[] getModulePositions() {
    SwerveModulePosition[] positions = new SwerveModulePosition[modules.length];
    for (int i = 0; i < modules.length; i++) {
      positions[i] = modules[i].getPosition();
    }
    return positions;
  }

  private SwerveSetpoint measuredSetpoint() {
    SwerveModuleState[] states = getModuleStates();
    return new SwerveSetpoint(
        DriveConstants.KINEMATICS.toChassisSpeeds(states),
        states,
        DriveFeedforwards.zeros(modules.length));
  }
}
