package frc.robot.subsystems.swervedrive;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Meters;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.RobotBase;
import frc.robot.Constants.VisionConstants;
import frc.robot.Constants.VisionConstants.CameraConfig;
import frc.robot.field.FieldConstants;
import java.util.ArrayList;
import java.util.List;
import java.util.Optional;
import org.littletonrobotics.junction.Logger;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;
import swervelib.SwerveDrive;

public class Vision {

  private static final String LOG_ROOT = "Vision/";

  private final List<Camera> cameras = new ArrayList<>();

  private final VisionSystemSim visionSim;

  private List<Pose2d> acceptedPoses = new ArrayList<>();

  private Pose2d averagePose = Pose2d.kZero;

  public Vision() {
    if (RobotBase.isSimulation()) {
      visionSim = new VisionSystemSim("main");
      visionSim.addAprilTags(FieldConstants.APRIL_TAG_LAYOUT);
    } else {
      visionSim = null;
    }
    for (CameraConfig config : VisionConstants.CAMERAS) {
      Camera camera = new Camera(config);
      cameras.add(camera);
      if (visionSim != null) {
        camera.cameraSim = new PhotonCameraSim(camera.photonCamera, simProperties());
        visionSim.addCamera(camera.cameraSim, config.robotToCamera());
      }
    }
  }

  public void updatePoseEstimation(SwerveDrive swerveDrive) {
    List<Pose2d> accepted = new ArrayList<>();
    List<Pose2d> rejected = new ArrayList<>();

    for (Camera camera : cameras) {
      camera.hasEstimate = false;
      for (PhotonPipelineResult result : camera.photonCamera.getAllUnreadResults()) {
        if (!result.hasTargets()) {
          continue;
        }
        PhotonTrackedTarget bestTarget = result.getBestTarget();
        if (bestTarget == null) {
          continue;
        }
        if (result.getTargets().size() == 1
            && bestTarget.getPoseAmbiguity() > VisionConstants.MAX_AMBIGUITY) {
          continue;
        }
        Optional<EstimatedRobotPose> estimate =
            camera.poseEstimator.estimateCoprocMultiTagPose(result);
        if (estimate.isEmpty()) {
          estimate = camera.poseEstimator.estimateLowestAmbiguityPose(result);
        }
        if (estimate.isEmpty()) {
          continue;
        }
        EstimatedRobotPose estimatedRobotPose = estimate.get();
        Pose3d estimatedPose = estimatedRobotPose.estimatedPose;
        Pose2d flatPose = estimatedPose.toPose2d();
        if (Math.abs(estimatedPose.getZ()) > VisionConstants.MAX_Z_ERROR.in(Meters)) {
          rejected.add(flatPose);
          continue;
        }
        if (!FieldConstants.isInsideField(flatPose)) {
          rejected.add(flatPose);
          continue;
        }
        camera.updateStdDevs(estimatedRobotPose);
        camera.latestPose = flatPose;
        camera.latestTimestamp = estimatedRobotPose.timestampSeconds;
        camera.hasEstimate = true;
        if (camera.curStdDevs.get(0, 0) >= Double.MAX_VALUE) {
          rejected.add(flatPose);
          break;
        }
        swerveDrive.addVisionMeasurement(
            flatPose, estimatedRobotPose.timestampSeconds, camera.curStdDevs);
        accepted.add(flatPose);
        break;
      }
      camera.log();
    }

    if (!accepted.isEmpty()) {
      averagePose = meanOf(accepted);
    }
    acceptedPoses = accepted;

    Logger.recordOutput(LOG_ROOT + "AcceptedPoses", accepted.toArray(new Pose2d[0]));
    Logger.recordOutput(LOG_ROOT + "RejectedPoses", rejected.toArray(new Pose2d[0]));
    Logger.recordOutput(LOG_ROOT + "AveragePose", averagePose);
    Logger.recordOutput(LOG_ROOT + "AcceptedCount", accepted.size());
    Logger.recordOutput(LOG_ROOT + "FusedPose", swerveDrive.getPose());
  }

  public void updateSimulation(Pose2d groundTruthPose) {
    if (visionSim == null || groundTruthPose == null) {
      return;
    }
    visionSim.update(groundTruthPose);
  }

  public List<Pose2d> getAcceptedPoses() {
    return List.copyOf(acceptedPoses);
  }

  private static SimCameraProperties simProperties() {
    SimCameraProperties properties = new SimCameraProperties();
    properties.setCalibration(
        VisionConstants.SIM_RESOLUTION_WIDTH,
        VisionConstants.SIM_RESOLUTION_HEIGHT,
        Rotation2d.fromDegrees(VisionConstants.SIM_DIAGONAL_FOV.in(Degrees)));
    properties.setCalibError(
        VisionConstants.SIM_CALIBRATION_ERROR_PIXELS,
        VisionConstants.SIM_CALIBRATION_ERROR_STD_DEV_PIXELS);
    properties.setFPS(VisionConstants.SIM_FRAMES_PER_SECOND);
    properties.setAvgLatencyMs(VisionConstants.SIM_AVERAGE_LATENCY_MILLIS);
    properties.setLatencyStdDevMs(VisionConstants.SIM_LATENCY_STD_DEV_MILLIS);
    return properties;
  }

  private static Pose2d meanOf(List<Pose2d> poses) {
    double sumX = 0.0;
    double sumY = 0.0;
    double sumCos = 0.0;
    double sumSin = 0.0;
    for (Pose2d pose : poses) {
      sumX += pose.getX();
      sumY += pose.getY();
      sumCos += pose.getRotation().getCos();
      sumSin += pose.getRotation().getSin();
    }
    double count = poses.size();
    return new Pose2d(sumX / count, sumY / count, new Rotation2d(sumCos / count, sumSin / count));
  }

  private class Camera {

    private final CameraConfig config;

    private final PhotonCamera photonCamera;

    private final PhotonPoseEstimator poseEstimator;

    private Matrix<N3, N1> curStdDevs = VisionConstants.SINGLE_TAG_STD_DEVS;

    private PhotonCameraSim cameraSim;

    private Pose2d latestPose = Pose2d.kZero;

    private double latestTimestamp;

    private int latestTagCount;

    private double latestAverageTagDistance;

    private boolean hasEstimate;

    private Camera(CameraConfig config) {
      this.config = config;
      this.photonCamera = new PhotonCamera(config.name());
      this.poseEstimator =
          new PhotonPoseEstimator(FieldConstants.APRIL_TAG_LAYOUT, config.robotToCamera());
    }

    private void updateStdDevs(EstimatedRobotPose estimate) {
      int numTags = 0;
      double avgDist = 0.0;
      for (PhotonTrackedTarget target : estimate.targetsUsed) {
        Optional<Pose3d> tagPose =
            FieldConstants.APRIL_TAG_LAYOUT.getTagPose(target.getFiducialId());
        if (tagPose.isEmpty()) {
          continue;
        }
        numTags++;
        avgDist +=
            tagPose.get().getTranslation().getDistance(estimate.estimatedPose.getTranslation());
      }
      latestTagCount = numTags;
      if (numTags == 0) {
        latestAverageTagDistance = 0.0;
        curStdDevs = VisionConstants.SINGLE_TAG_STD_DEVS;
        return;
      }
      avgDist /= numTags;
      latestAverageTagDistance = avgDist;
      Matrix<N3, N1> base =
          numTags > 1 ? VisionConstants.MULTI_TAG_STD_DEVS : VisionConstants.SINGLE_TAG_STD_DEVS;
      if (numTags == 1 && avgDist > VisionConstants.MAX_SINGLE_TAG_DISTANCE.in(Meters)) {
        curStdDevs = VecBuilder.fill(Double.MAX_VALUE, Double.MAX_VALUE, Double.MAX_VALUE);
      } else {
        curStdDevs = base.times(1 + (avgDist * avgDist / VisionConstants.STD_DEV_DISTANCE_DIVISOR));
        if (numTags == 1 && !VisionConstants.TRUST_SINGLE_TAG_ROTATION) {
          curStdDevs.set(2, 0, Double.MAX_VALUE);
        }
      }
    }

    private void log() {
      String prefix = LOG_ROOT + config.name() + "/";
      Logger.recordOutput(prefix + "EstimatedPose", latestPose);
      Logger.recordOutput(
          prefix + "StdDevs",
          new double[] {curStdDevs.get(0, 0), curStdDevs.get(1, 0), curStdDevs.get(2, 0)});
      Logger.recordOutput(prefix + "TagCount", latestTagCount);
      Logger.recordOutput(prefix + "AverageTagDistance", latestAverageTagDistance);
      Logger.recordOutput(prefix + "Timestamp", latestTimestamp);
      Logger.recordOutput(prefix + "Connected", photonCamera.isConnected());
      Logger.recordOutput(prefix + "HasEstimate", hasEstimate);
    }
  }
}
