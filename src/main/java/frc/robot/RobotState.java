package frc.robot;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import frc.robot.subsystems.drive.DriveConstants;
import java.util.Optional;
import org.littletonrobotics.junction.AutoLogOutput;

public final class RobotState {

  private static RobotState instance;

  public static RobotState getInstance() {
    if (instance == null) {
      instance = new RobotState();
    }
    return instance;
  }

  private final SwerveDrivePoseEstimator poseEstimator;

  private final SwerveDriveOdometry odometry;

  private Rotation2d lastGyroRotation = Rotation2d.kZero;

  private SwerveModulePosition[] lastModulePositions = emptyPositions();

  private ChassisSpeeds robotVelocity = new ChassisSpeeds();

  private RobotState() {
    double[] odometryStdDevs = DriveConstants.ODOMETRY_STD_DEVS;
    poseEstimator =
        new SwerveDrivePoseEstimator(
            DriveConstants.KINEMATICS,
            lastGyroRotation,
            lastModulePositions,
            Pose2d.kZero,
            VecBuilder.fill(odometryStdDevs[0], odometryStdDevs[1], odometryStdDevs[2]),
            VecBuilder.fill(0.9, 0.9, 0.9));
    odometry =
        new SwerveDriveOdometry(
            DriveConstants.KINEMATICS, lastGyroRotation, lastModulePositions, Pose2d.kZero);
  }

  public synchronized void addOdometryObservation(
      double timestampSeconds, Rotation2d gyroRotation, SwerveModulePosition[] modulePositions) {
    lastGyroRotation = gyroRotation;
    lastModulePositions = copyPositions(modulePositions);
    poseEstimator.updateWithTime(timestampSeconds, gyroRotation, lastModulePositions);
    odometry.update(gyroRotation, lastModulePositions);
  }

  public synchronized void addVisionObservation(
      Pose2d visionPose, double timestampSeconds, Matrix<N3, N1> standardDeviations) {
    poseEstimator.addVisionMeasurement(visionPose, timestampSeconds, standardDeviations);
  }

  public synchronized void addVelocityObservation(ChassisSpeeds measuredRobotRelativeSpeeds) {
    robotVelocity = measuredRobotRelativeSpeeds;
  }

  public synchronized void resetPose(Pose2d pose) {
    poseEstimator.resetPosition(lastGyroRotation, lastModulePositions, pose);
    odometry.resetPosition(lastGyroRotation, lastModulePositions, pose);
  }

  @AutoLogOutput(key = "RobotState/EstimatedPose")
  public synchronized Pose2d getPose() {
    return poseEstimator.getEstimatedPosition();
  }

  @AutoLogOutput(key = "RobotState/OdometryPose")
  public synchronized Pose2d getOdometryPose() {
    return odometry.getPoseMeters();
  }

  public synchronized Optional<Pose2d> getPoseAt(double timestampSeconds) {
    return poseEstimator.sampleAt(timestampSeconds);
  }

  public Rotation2d getRotation() {
    return getPose().getRotation();
  }

  @AutoLogOutput(key = "RobotState/RobotVelocity")
  public synchronized ChassisSpeeds getRobotVelocity() {
    return robotVelocity;
  }

  @AutoLogOutput(key = "RobotState/FieldVelocity")
  public synchronized ChassisSpeeds getFieldVelocity() {
    return ChassisSpeeds.fromRobotRelativeSpeeds(robotVelocity, getRotation());
  }

  private static SwerveModulePosition[] emptyPositions() {
    SwerveModulePosition[] positions = new SwerveModulePosition[DriveConstants.MODULES.length];
    for (int i = 0; i < positions.length; i++) {
      positions[i] = new SwerveModulePosition();
    }
    return positions;
  }

  private static SwerveModulePosition[] copyPositions(SwerveModulePosition[] positions) {
    SwerveModulePosition[] copy = new SwerveModulePosition[positions.length];
    for (int i = 0; i < positions.length; i++) {
      copy[i] = positions[i].copy();
    }
    return copy;
  }
}
