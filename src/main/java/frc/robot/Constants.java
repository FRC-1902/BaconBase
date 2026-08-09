package frc.robot;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.DegreesPerSecondPerSecond;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.MetersPerSecondPerSecond;
import static edu.wpi.first.units.Units.Milliseconds;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecondPerSecond;
import static edu.wpi.first.units.Units.Seconds;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearAcceleration;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj.RobotBase;

public final class Constants {

  private Constants() {}

  public enum RobotMode {
    REAL,
    SIM,
    REPLAY
  }

  public static final String PROJECT_NAME = "BaconBase";

  public static final RobotMode SIM_MODE = RobotMode.SIM;

  public static final RobotMode CURRENT_MODE = RobotBase.isReal() ? RobotMode.REAL : SIM_MODE;

  public static final class Drive {

    private Drive() {}

    public static final String SWERVE_CONFIG_DIRECTORY = "swerve/neo";

    public static final LinearVelocity MAX_SPEED = MetersPerSecond.of(Units.feetToMeters(14.5));

    public static final Pose2d STARTING_POSE = new Pose2d(3.0, 4.0, Rotation2d.kZero);

    public static final Time WHEEL_LOCK_TIME = Seconds.of(10);

    public static final boolean HEADING_CORRECTION = false;

    public static final boolean COSINE_COMPENSATION = false;

    public static final boolean ANGULAR_VELOCITY_COMPENSATION_TELEOP = true;

    public static final boolean ANGULAR_VELOCITY_COMPENSATION_AUTO = false;

    public static final double ANGULAR_VELOCITY_COEFFICIENT = 0.025;

    public static final boolean CHASSIS_DISCRETIZATION = true;

    public static final double CHASSIS_DISCRETIZATION_SECONDS = 0.02;

    public static final boolean MODULE_ENCODER_AUTO_SYNCHRONIZE = false;

    public static final double MODULE_ENCODER_AUTO_SYNCHRONIZE_DEADBAND = 1.0;
  }

  public static final class Operator {

    private Operator() {}

    public static final int DRIVE_CONTROLLER_PORT = 0;

    public static final int MANIP_CONTROLLER_PORT = 1;

    public static final double TRANSLATION_DEADBAND = 0.1;

    public static final double ROTATION_DEADBAND = 0.1;

    public static final double TRANSLATION_SCALE = 1.0;

    public static final double ROTATION_SCALE = 0.4;

    public static final double INPUT_LINEAR_WEIGHT = 0.8;

    public static final double INPUT_CUBIC_WEIGHT = 0.2;

    public static final long RUMBLE_DURATION_MILLIS = 500;

    public static final double RUMBLE_INTENSITY = 1.0;
  }

  public static final class Alignment {

    private Alignment() {}

    public static final double SNAP_TRANSLATION_P = 5.0;

    public static final double SNAP_TRANSLATION_I = 0.0;

    public static final double SNAP_TRANSLATION_D = 0.0;

    public static final LinearVelocity SNAP_MAX_VELOCITY = MetersPerSecond.of(3.0);

    public static final LinearAcceleration SNAP_MAX_ACCELERATION = MetersPerSecondPerSecond.of(3.0);

    public static final double SNAP_ROTATION_P = 4.0;

    public static final double SNAP_ROTATION_I = 0.0;

    public static final double SNAP_ROTATION_D = 0.0;

    public static final AngularVelocity SNAP_MAX_ANGULAR_VELOCITY = DegreesPerSecond.of(540);

    public static final AngularAcceleration SNAP_MAX_ANGULAR_ACCELERATION =
        DegreesPerSecondPerSecond.of(720);

    public static final Distance SNAP_TRANSLATION_TOLERANCE = Meters.of(0.05);

    public static final Angle SNAP_ROTATION_TOLERANCE = Degrees.of(3);

    public static final Time SNAP_TIMEOUT = Seconds.of(3);

    public static final double AIM_TRANSLATION_SCALE = 0.3;

    public static final Time AIM_LOOKAHEAD = Milliseconds.of(500);

    public static final Angle AIM_LOCK_TOLERANCE = Degrees.of(3);

    public static ProfiledPIDController translationController() {
      return new ProfiledPIDController(
          SNAP_TRANSLATION_P,
          SNAP_TRANSLATION_I,
          SNAP_TRANSLATION_D,
          new TrapezoidProfile.Constraints(
              SNAP_MAX_VELOCITY.in(MetersPerSecond),
              SNAP_MAX_ACCELERATION.in(MetersPerSecondPerSecond)));
    }

    public static ProfiledPIDController rotationController() {
      ProfiledPIDController controller =
          new ProfiledPIDController(
              SNAP_ROTATION_P,
              SNAP_ROTATION_I,
              SNAP_ROTATION_D,
              new TrapezoidProfile.Constraints(
                  SNAP_MAX_ANGULAR_VELOCITY.in(RadiansPerSecond),
                  SNAP_MAX_ANGULAR_ACCELERATION.in(RadiansPerSecondPerSecond)));
      controller.enableContinuousInput(-Math.PI, Math.PI);
      return controller;
    }
  }

  public static final class Auto {

    private Auto() {}

    public static final double TRANSLATION_P = 5.0;

    public static final double TRANSLATION_I = 0.0;

    public static final double TRANSLATION_D = 0.0;

    public static final double ROTATION_P = 5.0;

    public static final double ROTATION_I = 0.0;

    public static final double ROTATION_D = 0.0;

    public static final LinearVelocity PATHFIND_MAX_VELOCITY = MetersPerSecond.of(3.0);

    public static final LinearAcceleration PATHFIND_MAX_ACCELERATION =
        MetersPerSecondPerSecond.of(3.0);

    public static final AngularVelocity PATHFIND_MAX_ANGULAR_VELOCITY = DegreesPerSecond.of(540);

    public static final AngularAcceleration PATHFIND_MAX_ANGULAR_ACCELERATION =
        DegreesPerSecondPerSecond.of(720);

    public static final LinearVelocity PATHFIND_GOAL_END_VELOCITY = MetersPerSecond.of(0);

    public static final String DEFAULT_AUTO_NAME = "Do Nothing";
  }

  public static final class VisionConstants {

    private VisionConstants() {}

    public record CameraConfig(String name, Transform3d robotToCamera) {}

    public static final Matrix<N3, N1> SINGLE_TAG_STD_DEVS = VecBuilder.fill(0.3, 0.3, 0.5);

    public static final Matrix<N3, N1> MULTI_TAG_STD_DEVS = VecBuilder.fill(0.1, 0.1, 0.2);

    public static final double MAX_AMBIGUITY = 0.2;

    public static final Distance MAX_SINGLE_TAG_DISTANCE = Meters.of(4.0);

    public static final double STD_DEV_DISTANCE_DIVISOR = 30.0;

    public static final boolean TRUST_SINGLE_TAG_ROTATION = false;

    public static final Distance MAX_Z_ERROR = Meters.of(0.5);

    public static final CameraConfig FRONT_LEFT =
        new CameraConfig(
            "frontLeft",
            new Transform3d(
                new Translation3d(
                    Units.inchesToMeters(10.0),
                    Units.inchesToMeters(10.0),
                    Units.inchesToMeters(8.0)),
                new Rotation3d(0.0, Math.toRadians(-20.0), Math.toRadians(30.0))));

    public static final CameraConfig FRONT_RIGHT =
        new CameraConfig(
            "frontRight",
            new Transform3d(
                new Translation3d(
                    Units.inchesToMeters(10.0),
                    Units.inchesToMeters(-10.0),
                    Units.inchesToMeters(8.0)),
                new Rotation3d(0.0, Math.toRadians(-20.0), Math.toRadians(-30.0))));

    public static final CameraConfig BACK_LEFT =
        new CameraConfig(
            "backLeft",
            new Transform3d(
                new Translation3d(
                    Units.inchesToMeters(-10.0),
                    Units.inchesToMeters(10.0),
                    Units.inchesToMeters(8.0)),
                new Rotation3d(0.0, Math.toRadians(-20.0), Math.toRadians(150.0))));

    public static final CameraConfig BACK_RIGHT =
        new CameraConfig(
            "backRight",
            new Transform3d(
                new Translation3d(
                    Units.inchesToMeters(-10.0),
                    Units.inchesToMeters(-10.0),
                    Units.inchesToMeters(8.0)),
                new Rotation3d(0.0, Math.toRadians(-20.0), Math.toRadians(-150.0))));

    public static final CameraConfig[] CAMERAS = {FRONT_LEFT, FRONT_RIGHT, BACK_LEFT, BACK_RIGHT};

    public static final int SIM_RESOLUTION_WIDTH = 960;

    public static final int SIM_RESOLUTION_HEIGHT = 720;

    public static final Angle SIM_DIAGONAL_FOV = Degrees.of(70);

    public static final double SIM_CALIBRATION_ERROR_PIXELS = 0.25;

    public static final double SIM_CALIBRATION_ERROR_STD_DEV_PIXELS = 0.08;

    public static final double SIM_FRAMES_PER_SECOND = 30.0;

    public static final double SIM_AVERAGE_LATENCY_MILLIS = 35.0;

    public static final double SIM_LATENCY_STD_DEV_MILLIS = 5.0;
  }

  public static final class Power {

    private Power() {}

    public static final int PDH_ID = 1;

    public static final double BATTERY_WARNING_VOLTAGE = 12.0;

    public static final double BATTERY_CRITICAL_VOLTAGE = 11.5;

    public static final Time BATTERY_CHECK_INTERVAL = Seconds.of(5);
  }
}
