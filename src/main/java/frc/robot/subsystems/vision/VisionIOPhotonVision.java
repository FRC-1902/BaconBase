package frc.robot.subsystems.vision;

import static edu.wpi.first.units.Units.Meters;

import edu.wpi.first.wpilibj.Timer;
import frc.robot.RobotState;
import frc.robot.field.FieldConstants;
import frc.robot.subsystems.vision.VisionConstants.CameraConfig;
import frc.robot.util.LoggedTunableNumber;
import java.util.ArrayList;
import java.util.List;
import java.util.Optional;
import java.util.Set;
import java.util.TreeSet;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.targeting.MultiTargetPNPResult;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

public class VisionIOPhotonVision implements VisionIO {

  private static final LoggedTunableNumber trigSolveMaxDistance =
      new LoggedTunableNumber(
          "Vision/TrigSolveMaxDistance", VisionConstants.TRIG_SOLVE_MAX_DISTANCE.in(Meters));

  protected final PhotonCamera camera;

  private final PhotonPoseEstimator poseEstimator;

  public VisionIOPhotonVision(CameraConfig config) {
    camera = new PhotonCamera(config.name());
    poseEstimator =
        new PhotonPoseEstimator(FieldConstants.APRIL_TAG_LAYOUT, config.robotToCamera());
  }

  @Override
  public void updateInputs(VisionIOInputs inputs) {
    inputs.connected = camera.isConnected();
    poseEstimator.addHeadingData(Timer.getFPGATimestamp(), RobotState.getInstance().getRotation());

    Set<Integer> tagIds = new TreeSet<>();
    List<PoseObservation> poseObservations = new ArrayList<>();
    for (PhotonPipelineResult result : camera.getAllUnreadResults()) {
      for (PhotonTrackedTarget target : result.getTargets()) {
        if (target.getFiducialId() >= 0) {
          tagIds.add(target.getFiducialId());
        }
      }
      estimate(result).ifPresent(poseObservations::add);
    }

    inputs.poseObservations = poseObservations.toArray(new PoseObservation[0]);
    inputs.tagIds = tagIds.stream().mapToInt(Integer::intValue).toArray();
  }

  private Optional<PoseObservation> estimate(PhotonPipelineResult result) {
    if (!result.hasTargets()) {
      return Optional.empty();
    }

    Optional<MultiTargetPNPResult> multiTag = result.getMultiTagResult();
    if (multiTag.isPresent()) {
      return estimateMultiTag(result, multiTag.get());
    }

    PhotonTrackedTarget bestTarget = result.getBestTarget();
    if (distance(bestTarget) <= trigSolveMaxDistance.get()) {
      Optional<PoseObservation> trigSolve =
          poseEstimator
              .estimatePnpDistanceTrigSolvePose(result)
              .map(
                  estimate ->
                      observation(
                          estimate,
                          bestTarget.getPoseAmbiguity(),
                          1,
                          distance(bestTarget),
                          PoseObservationType.TRIG_SOLVE));
      if (trigSolve.isPresent()) {
        return trigSolve;
      }
    }

    return estimateSingleTag(result);
  }

  private Optional<PoseObservation> estimateMultiTag(
      PhotonPipelineResult result, MultiTargetPNPResult multiTag) {
    double usedDistance = 0.0;
    int usedTargets = 0;
    double allDistance = 0.0;
    for (PhotonTrackedTarget target : result.getTargets()) {
      allDistance += distance(target);
      if (multiTag.fiducialIDsUsed.contains((short) target.getFiducialId())) {
        usedDistance += distance(target);
        usedTargets++;
      }
    }
    double averageDistance =
        usedTargets > 0 ? usedDistance / usedTargets : allDistance / result.getTargets().size();
    return poseEstimator
        .estimateCoprocMultiTagPose(result)
        .map(
            estimate ->
                observation(
                    estimate,
                    multiTag.estimatedPose.ambiguity,
                    multiTag.fiducialIDsUsed.size(),
                    averageDistance,
                    PoseObservationType.MULTI_TAG));
  }

  private Optional<PoseObservation> estimateSingleTag(PhotonPipelineResult result) {
    PhotonTrackedTarget lowestAmbiguityTarget = null;
    for (PhotonTrackedTarget target : result.getTargets()) {
      double ambiguity = target.getPoseAmbiguity();
      if (ambiguity != -1
          && (lowestAmbiguityTarget == null
              || ambiguity < lowestAmbiguityTarget.getPoseAmbiguity())) {
        lowestAmbiguityTarget = target;
      }
    }
    if (lowestAmbiguityTarget == null) {
      return Optional.empty();
    }
    PhotonTrackedTarget target = lowestAmbiguityTarget;
    return poseEstimator
        .estimateLowestAmbiguityPose(result)
        .map(
            estimate ->
                observation(
                    estimate,
                    target.getPoseAmbiguity(),
                    1,
                    distance(target),
                    PoseObservationType.SINGLE_TAG));
  }

  private static PoseObservation observation(
      EstimatedRobotPose estimate,
      double ambiguity,
      int tagCount,
      double averageTagDistance,
      PoseObservationType type) {
    return new PoseObservation(
        estimate.timestampSeconds,
        estimate.estimatedPose,
        ambiguity,
        tagCount,
        averageTagDistance,
        type);
  }

  private static double distance(PhotonTrackedTarget target) {
    return target.getBestCameraToTarget().getTranslation().getNorm();
  }
}
