package frc.robot.subsystems.vision;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;

public final class VisionConstants {

  private VisionConstants() {}

  public record CameraConfig(String name, Transform3d robotToCamera, double stdDevFactor) {}

  // TODO: CAMERA NAMES
  // How to set up:
  //   1. Open the PhotonVision UI: http://photonvision.local:5800 or http://10.19.2.200:5800 (may
  // be 201/202 for other orangepis)
  //   2. Cameras tab -> select each camera -> set its name to frontLeft / frontRight / backLeft /
  //      backRight (or change the names here to match). Unplug cameras one at a time to be sure
  //      which physical camera is which name.
  //   3. Deploy, enable nothing, and check "Vision/Camera/<name>/Connected" is true for all four
  //      and that there is no "Vision camera '<name>' is disconnected" alert on the dashboard.

  // TODO: CAMERA CALIBRATION (done in the PhotonVision UI)
  // How to set up:
  //   1. Cameras tab -> set the exact resolution the AprilTag pipeline will use.
  //   2. Calibration -> print the ChArUco board at 100% scale on a flat rigid surface and enter
  //      the square and marker sizes measured with calipers.
  //   3. Take 30-50 snapshots: board in every part of the image, at different distances, tilted up
  //      to ~45 degrees. Hold still for each one.
  //   4. Accept only if mean reprojection error is under ~0.5 px. Otherwise delete the worst
  //      snapshots and recalibrate.
  //   5. Recalibrate if the lens is touched or the camera is replaced.

  // TODO: PHOTONVISION PIPELINE SETUP (done in the PhotonVision UI)
  // How to set up:
  //   1. Pipeline type AprilTag, family 36h11, 3D mode.
  //   2. Output tab -> turn on "Do Multi-Target Estimation".
  //   3. Use the same field layout as FieldConstants.APRIL_TAG_LAYOUT (Settings -> AprilTag Field
  //      Layout).
  //   4. Lower exposure until tags are only just detected at your longest useful distance. Raise
  //      gain/brightness to compensate.
  //   5. Write down each camera's FPS and latency for the SIM_* constants below.
  //   6. Re-do this at every competition during field-cal time.

  // TODO: ROBOT-TO-CAMERA TRANSFORMS
  // How to measure (robot on flat floor, one camera at a time):
  //   1. Axes: origin is the robot center on the floor, +X forward, +Y left, +Z up, in meters.
  //      Rotation3d is (roll, pitch, yaw): yaw +30 = looking forward-left, back-left corner is
  //      +150, back-right is -150. NEGATIVE pitch = camera tilted UP.
  //   2. Translation3d: from CAD or a tape measure, X and Y from the middle of the four modules
  //      to the lens, Z = lens height above the floor. Pitch from a digital angle gauge, yaw from
  //      CAD or a protractor.
  //   3. Static check: place the robot at a tape-measured pose (e.g. bumper flush on the wall,
  //      centered on a tag) and compare "Vision/Camera/<name>/RobotPoses" to it. Fix X/Y/yaw until
  //      within ~2 cm and ~1 degree at 1-3 m.
  //   4. Spin check: with 2+ tags in view, spin slowly in place. RobotPoses should stay on one
  //      spot. A circle means X/Y is wrong. A constant heading offset from the gyro means yaw is
  //      wrong. Poses too close/far or Z not ~0 means pitch or Z is wrong.
  //   5. Put all cameras back. Where two cameras see tags, their RobotPoses must agree within
  //      ~2-3 cm. Re-measure the one that disagrees.

  public static final CameraConfig FRONT_LEFT =
      new CameraConfig(
          "frontLeft",
          new Transform3d(
              new Translation3d(Inches.of(10.0), Inches.of(10.0), Inches.of(8.0)),
              new Rotation3d(Degrees.of(0.0), Degrees.of(-20.0), Degrees.of(30.0))),
          1.0);

  public static final CameraConfig FRONT_RIGHT =
      new CameraConfig(
          "frontRight",
          new Transform3d(
              new Translation3d(Inches.of(10.0), Inches.of(-10.0), Inches.of(8.0)),
              new Rotation3d(Degrees.of(0.0), Degrees.of(-20.0), Degrees.of(-30.0))),
          1.0);

  public static final CameraConfig BACK_LEFT =
      new CameraConfig(
          "backLeft",
          new Transform3d(
              new Translation3d(Inches.of(-10.0), Inches.of(10.0), Inches.of(8.0)),
              new Rotation3d(Degrees.of(0.0), Degrees.of(-20.0), Degrees.of(150.0))),
          1.0);

  public static final CameraConfig BACK_RIGHT =
      new CameraConfig(
          "backRight",
          new Transform3d(
              new Translation3d(Inches.of(-10.0), Inches.of(-10.0), Inches.of(8.0)),
              new Rotation3d(Degrees.of(0.0), Degrees.of(-20.0), Degrees.of(-150.0))),
          1.0);

  public static final CameraConfig[] CAMERAS = {FRONT_LEFT, FRONT_RIGHT, BACK_LEFT, BACK_RIGHT};

  // TODO: MAX AMBIGUITY (0 to 1)
  // How to tune:
  //   1. Drive slowly around the field 2-4 m from tags, often seeing only one tag, including steep
  //      side angles.
  //   2. Watch RobotPosesAccepted and RobotPosesRejected (reason AMBIGUITY) in the 3D field, and
  //      plot ambiguity in "Vision/Camera/<name>/PoseObservations".
  //   3. If accepted poses jump far away for a frame, lower it by 0.05.
  //   4. If rejected AMBIGUITY poses look correct, raise it by 0.05 (max ~0.3).
  public static final double MAX_AMBIGUITY = 0.2;

  public static final Distance MAX_Z_ERROR = Meters.of(0.5);

  public static final Distance FIELD_BORDER_MARGIN = Meters.of(0.5);

  public static final Distance MAX_SINGLE_TAG_DISTANCE = Meters.of(4.0);

  public static final Distance TRIG_SOLVE_MAX_DISTANCE = Meters.of(2.0);

  public static final AngularVelocity MAX_ANGULAR_VELOCITY = RadiansPerSecond.of(4.0);

  public static final Distance LINEAR_STD_DEV_BASELINE = Meters.of(0.02);

  public static final Angle ANGULAR_STD_DEV_BASELINE = Radians.of(0.06);

  public static final double TRIG_SOLVE_STD_DEV_FACTOR = 0.5;

  public static final int SIM_RESOLUTION_WIDTH = 960;

  public static final int SIM_RESOLUTION_HEIGHT = 720;

  public static final Angle SIM_DIAGONAL_FOV = Degrees.of(70);

  public static final double SIM_CALIBRATION_ERROR_PIXELS = 0.25;

  public static final double SIM_CALIBRATION_ERROR_STD_DEV_PIXELS = 0.08;

  public static final double SIM_FRAMES_PER_SECOND = 30.0;

  public static final double SIM_AVERAGE_LATENCY_MILLIS = 35.0;

  public static final double SIM_LATENCY_STD_DEV_MILLIS = 5.0;

  public static final boolean SIM_VIDEO_STREAMS_ENABLED = false;
}
