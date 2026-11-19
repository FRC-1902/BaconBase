package frc.robot.subsystems.vision;

import static edu.wpi.first.units.Units.Degrees;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.field.FieldConstants;
import frc.robot.subsystems.vision.VisionConstants.CameraConfig;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;

public class VisionIOPhotonVisionSim extends VisionIOPhotonVision {

  private static VisionSystemSim visionSim;

  private static long lastSimUpdateTimestamp = Long.MIN_VALUE;

  private final Supplier<Pose2d> groundTruthPoseSupplier;

  public VisionIOPhotonVisionSim(CameraConfig config, Supplier<Pose2d> groundTruthPoseSupplier) {
    super(config);
    this.groundTruthPoseSupplier = groundTruthPoseSupplier;

    if (visionSim == null) {
      visionSim = new VisionSystemSim("main");
      visionSim.addAprilTags(FieldConstants.APRIL_TAG_LAYOUT);
    }

    PhotonCameraSim cameraSim =
        new PhotonCameraSim(camera, cameraProperties(), FieldConstants.APRIL_TAG_LAYOUT);
    cameraSim.enableRawStream(VisionConstants.SIM_VIDEO_STREAMS_ENABLED);
    cameraSim.enableProcessedStream(VisionConstants.SIM_VIDEO_STREAMS_ENABLED);
    visionSim.addCamera(cameraSim, config.robotToCamera());
  }

  public static void resetSimulationPose(Pose2d pose) {
    if (visionSim != null) {
      visionSim.resetRobotPose(pose);
    }
  }

  @Override
  public void updateInputs(VisionIOInputs inputs) {
    long timestamp = Logger.getTimestamp();
    if (timestamp != lastSimUpdateTimestamp) {
      lastSimUpdateTimestamp = timestamp;
      visionSim.update(groundTruthPoseSupplier.get());
    }
    super.updateInputs(inputs);
  }

  private static SimCameraProperties cameraProperties() {
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
}
