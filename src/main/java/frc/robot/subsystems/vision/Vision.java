package frc.robot.subsystems.vision;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotState;
import frc.robot.field.FieldConstants;
import frc.robot.subsystems.vision.VisionConstants.CameraConfig;
import frc.robot.subsystems.vision.VisionIO.PoseObservation;
import frc.robot.subsystems.vision.VisionIO.PoseObservationType;
import frc.robot.util.LoggedTunableNumber;
import java.util.ArrayList;
import java.util.Comparator;
import java.util.List;
import java.util.Optional;
import org.littletonrobotics.junction.Logger;

public class Vision extends SubsystemBase {

  private static final String LOG_ROOT = "Vision/";

  private static final LoggedTunableNumber maxAmbiguity =
      new LoggedTunableNumber("Vision/MaxAmbiguity", VisionConstants.MAX_AMBIGUITY);

  private static final LoggedTunableNumber maxZError =
      new LoggedTunableNumber("Vision/MaxZError", VisionConstants.MAX_Z_ERROR.in(Meters));

  private static final LoggedTunableNumber fieldBorderMargin =
      new LoggedTunableNumber(
          "Vision/FieldBorderMargin", VisionConstants.FIELD_BORDER_MARGIN.in(Meters));

  private static final LoggedTunableNumber maxSingleTagDistance =
      new LoggedTunableNumber(
          "Vision/MaxSingleTagDistance", VisionConstants.MAX_SINGLE_TAG_DISTANCE.in(Meters));

  private static final LoggedTunableNumber maxAngularVelocity =
      new LoggedTunableNumber(
          "Vision/MaxAngularVelocity", VisionConstants.MAX_ANGULAR_VELOCITY.in(RadiansPerSecond));

  private static final LoggedTunableNumber linearStdDevBaseline =
      new LoggedTunableNumber(
          "Vision/LinearStdDevBaseline", VisionConstants.LINEAR_STD_DEV_BASELINE.in(Meters));

  private static final LoggedTunableNumber angularStdDevBaseline =
      new LoggedTunableNumber(
          "Vision/AngularStdDevBaseline", VisionConstants.ANGULAR_STD_DEV_BASELINE.in(Radians));

  private static final LoggedTunableNumber trigSolveStdDevFactor =
      new LoggedTunableNumber(
          "Vision/TrigSolveStdDevFactor", VisionConstants.TRIG_SOLVE_STD_DEV_FACTOR);

  private enum Rejection {
    NO_TAGS,
    AMBIGUITY,
    Z_ERROR,
    OUT_OF_FIELD,
    TOO_FAR,
    SPINNING
  }

  private record Measurement(Pose2d pose, double timestamp, Matrix<N3, N1> stdDevs) {}

  private final CameraConfig[] cameras;

  private final VisionIO[] io;

  private final VisionIOInputsAutoLogged[] inputs;

  private final Alert[] disconnectedAlerts;

  public Vision(VisionIO... io) {
    if (io.length != VisionConstants.CAMERAS.length) {
      throw new IllegalArgumentException(
          "Vision needs exactly one VisionIO per VisionConstants.CAMERAS entry, in the same order"
              + " (got "
              + io.length
              + ", expected "
              + VisionConstants.CAMERAS.length
              + ")");
    }
    this.cameras = VisionConstants.CAMERAS;
    this.io = io;
    this.inputs = new VisionIOInputsAutoLogged[io.length];
    this.disconnectedAlerts = new Alert[io.length];
    for (int i = 0; i < io.length; i++) {
      inputs[i] = new VisionIOInputsAutoLogged();
      disconnectedAlerts[i] =
          new Alert(
              "Vision camera '" + cameras[i].name() + "' is disconnected.", AlertType.kWarning);
    }
  }

  @Override
  public void periodic() {
    if (!VisionConstants.ENABLED) {
      return;
    }
    for (int i = 0; i < io.length; i++) {
      io[i].updateInputs(inputs[i]);
      Logger.processInputs(cameraKey(i), inputs[i]);
    }

    double angularVelocity =
        Math.abs(RobotState.getInstance().getRobotVelocity().omegaRadiansPerSecond);

    List<Measurement> measurements = new ArrayList<>();
    List<Pose3d> allTagPoses = new ArrayList<>();
    List<Pose3d> allRobotPoses = new ArrayList<>();
    List<Pose3d> allRobotPosesAccepted = new ArrayList<>();
    List<Pose3d> allRobotPosesRejected = new ArrayList<>();

    for (int i = 0; i < io.length; i++) {
      disconnectedAlerts[i].set(!inputs[i].connected);

      List<Pose3d> tagPoses = new ArrayList<>();
      List<Pose3d> robotPoses = new ArrayList<>();
      List<Pose3d> robotPosesAccepted = new ArrayList<>();
      List<Pose3d> robotPosesRejected = new ArrayList<>();
      List<String> rejectionReasons = new ArrayList<>();
      List<Double> acceptedLinearStdDevs = new ArrayList<>();

      for (int tagId : inputs[i].tagIds) {
        FieldConstants.APRIL_TAG_LAYOUT.getTagPose(tagId).ifPresent(tagPoses::add);
      }

      for (PoseObservation observation : inputs[i].poseObservations) {
        robotPoses.add(observation.pose());
        Optional<Rejection> rejection = rejection(observation, angularVelocity);
        if (rejection.isPresent()) {
          robotPosesRejected.add(observation.pose());
          rejectionReasons.add(rejection.get().name());
          continue;
        }
        Matrix<N3, N1> stdDevs = stdDevs(observation, cameras[i]);
        robotPosesAccepted.add(observation.pose());
        acceptedLinearStdDevs.add(stdDevs.get(0, 0));
        measurements.add(
            new Measurement(observation.pose().toPose2d(), observation.timestamp(), stdDevs));
      }

      String key = cameraKey(i);
      Logger.recordOutput(key + "/TagPoses", tagPoses.toArray(Pose3d[]::new));
      Logger.recordOutput(key + "/RobotPoses", robotPoses.toArray(Pose3d[]::new));
      Logger.recordOutput(key + "/RobotPosesAccepted", robotPosesAccepted.toArray(Pose3d[]::new));
      Logger.recordOutput(key + "/RobotPosesRejected", robotPosesRejected.toArray(Pose3d[]::new));
      Logger.recordOutput(key + "/RejectionReasons", rejectionReasons.toArray(String[]::new));
      Logger.recordOutput(
          key + "/AcceptedLinearStdDevs",
          acceptedLinearStdDevs.stream().mapToDouble(Double::doubleValue).toArray());

      allTagPoses.addAll(tagPoses);
      allRobotPoses.addAll(robotPoses);
      allRobotPosesAccepted.addAll(robotPosesAccepted);
      allRobotPosesRejected.addAll(robotPosesRejected);
    }

    measurements.sort(Comparator.comparingDouble(Measurement::timestamp));
    for (Measurement measurement : measurements) {
      RobotState.getInstance()
          .addVisionObservation(measurement.pose(), measurement.timestamp(), measurement.stdDevs());
    }

    Logger.recordOutput(LOG_ROOT + "Summary/TagPoses", allTagPoses.toArray(Pose3d[]::new));
    Logger.recordOutput(LOG_ROOT + "Summary/RobotPoses", allRobotPoses.toArray(Pose3d[]::new));
    Logger.recordOutput(
        LOG_ROOT + "Summary/RobotPosesAccepted", allRobotPosesAccepted.toArray(Pose3d[]::new));
    Logger.recordOutput(
        LOG_ROOT + "Summary/RobotPosesRejected", allRobotPosesRejected.toArray(Pose3d[]::new));
  }

  private String cameraKey(int index) {
    return LOG_ROOT + "Camera/" + cameras[index].name();
  }

  private static Optional<Rejection> rejection(
      PoseObservation observation, double angularVelocity) {
    Pose3d pose = observation.pose();
    double margin = fieldBorderMargin.get();
    if (observation.tagCount() == 0) {
      return Optional.of(Rejection.NO_TAGS);
    }
    if (observation.type() == PoseObservationType.SINGLE_TAG
        && observation.ambiguity() > maxAmbiguity.get()) {
      return Optional.of(Rejection.AMBIGUITY);
    }
    if (observation.type() != PoseObservationType.TRIG_SOLVE
        && Math.abs(pose.getZ()) > maxZError.get()) {
      return Optional.of(Rejection.Z_ERROR);
    }
    if (pose.getX() < -margin
        || pose.getX() > FieldConstants.FIELD_LENGTH + margin
        || pose.getY() < -margin
        || pose.getY() > FieldConstants.FIELD_WIDTH + margin) {
      return Optional.of(Rejection.OUT_OF_FIELD);
    }
    if (observation.type() == PoseObservationType.SINGLE_TAG
        && observation.averageTagDistance() > maxSingleTagDistance.get()) {
      return Optional.of(Rejection.TOO_FAR);
    }
    if (angularVelocity > maxAngularVelocity.get()) {
      return Optional.of(Rejection.SPINNING);
    }
    return Optional.empty();
  }

  private static Matrix<N3, N1> stdDevs(PoseObservation observation, CameraConfig camera) {
    double factor =
        observation.averageTagDistance()
            * observation.averageTagDistance()
            / observation.tagCount()
            * camera.stdDevFactor();
    double linear = linearStdDevBaseline.get() * factor;
    double angular = angularStdDevBaseline.get() * factor;
    if (observation.type() != PoseObservationType.MULTI_TAG) {
      angular = Double.POSITIVE_INFINITY;
    }
    if (observation.type() == PoseObservationType.TRIG_SOLVE) {
      linear *= trigSolveStdDevFactor.get();
    }
    return VecBuilder.fill(linear, linear, angular);
  }
}
