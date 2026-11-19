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

  public static final double LOOP_PERIOD_SECONDS = 0.02;

  // TODO: TUNING MODE
  // How to use:
  //   1. Set to true and deploy.
  //   2. Open Elastic or AdvantageScope -> NetworkTables -> "/Tuning" and edit values live.
  //   3. Copy every final value back into the code (dashboard values are lost on reboot).
  //   4. Set back to false before competition.
  public static final boolean TUNING_MODE = false;

  public static final class Operator {

    private Operator() {}

    public static final int DRIVE_CONTROLLER_PORT = 0;

    public static final int MANIP_CONTROLLER_PORT = 1;

    // TODO: STICK DEADBANDS (0 to 1)
    // How to tune:
    //   1. Enable teleop and let go of both sticks.
    //   2. In AdvantageScope plot "Driver/RawLeftX", "Driver/RawLeftY", "Driver/RawRightX".
    //   3. Set each deadband 0.02 above the largest value the stick rests at.
    public static final double TRANSLATION_DEADBAND = 0.1;

    public static final double ROTATION_DEADBAND = 0.1;

    public static final double TRANSLATION_SCALE = 1.0;

    public static final double ROTATION_SCALE = 0.4;

    // TODO: STICK RESPONSE CURVE (the two weights must add up to 1.0)
    // How to tune:
    //   1. Have the driver line up on a target using only the sticks.
    //   2. If small precise moves are hard, move weight to cubic (e.g. 0.6 / 0.4).
    //   3. If the robot feels laggy to start moving, move weight to linear (e.g. 0.9 / 0.1).
    public static final double INPUT_LINEAR_WEIGHT = 0.8;

    public static final double INPUT_CUBIC_WEIGHT = 0.2;

    // TODO: HEADING HOLD
    // How to tune ("/Tuning/Teleop/HeadingHoldKp", "/Tuning/Teleop/HeadingHoldDelaySeconds"):
    //   1. Drive full speed straight forward.
    //   2. Plot "Teleop/HeadingHoldTarget" and the rotation of "RobotState/EstimatedPose".
    //   3. If heading drifts more than 2 deg, raise kP by 1.0. If it wiggles, lower kP by 1.0.
    //   4. If it snaps back after you stop spinning, raise the delay by 0.05 s.
    //   5. Set HEADING_HOLD_ENABLED = false if the driver doesn't want it.
    public static final boolean HEADING_HOLD_ENABLED = true;

    public static final double HEADING_HOLD_KP = 4.0;

    public static final Time HEADING_HOLD_DELAY = Seconds.of(0.25);

    public static final long RUMBLE_DURATION_MILLIS = 500;

    public static final double RUMBLE_INTENSITY = 1.0;
  }

  public static final class Alignment {

    private Alignment() {}

    // TODO: DRIVE-TO-POSE TRANSLATION
    // How to tune ("/Tuning/DriveToPose/..."):
    //   1. Start 2 m from a waypoint and hold the align button.
    //   2. Plot "DriveToPose/DistanceError", "DriveToPose/DistanceSetpoint", "DriveToPose/AtGoal".
    //   3. Raise max velocity/acceleration until the robot slides, then lower them 20%.
    //   4. Raise kP by 0.5 until it overshoots or hunts at the end, then lower it by 1.0.
    //   5. Only add kD (steps of 0.05) if overshoot remains. Lower it if the robot buzzes.
    //   6. Run 10 aligns from different sides; final error must be under TRANSLATION_TOLERANCE.
    public static final double TRANSLATION_KP = 4.0;

    public static final double TRANSLATION_KD = 0.0;

    public static final LinearVelocity TRANSLATION_MAX_VELOCITY = MetersPerSecond.of(3.5);

    public static final LinearAcceleration TRANSLATION_MAX_ACCELERATION =
        MetersPerSecondPerSecond.of(5.0);

    // TODO: DRIVE-TO-POSE ROTATION
    // How to tune ("/Tuning/DriveToPose/Rotation..."):
    //   1. Same setup as translation, plot "DriveToPose/RotationError".
    //   2. Raise kP by 0.5 until the final heading overshoots, then lower it by 1.0.
    //   3. If rotation finishes long after translation, raise ROTATION_MAX_ACCELERATION.
    public static final double ROTATION_KP = 4.0;

    public static final double ROTATION_KD = 0.0;

    public static final AngularVelocity ROTATION_MAX_VELOCITY = DegreesPerSecond.of(500);

    public static final AngularAcceleration ROTATION_MAX_ACCELERATION =
        RadiansPerSecondPerSecond.of(20.0);

    // TODO: FEEDFORWARD FADE RADII (meters)
    // How to tune:
    //   1. Watch the end of an align in "DriveToPose/DistanceError".
    //   2. If it overshoots by a few cm and comes back, raise FF_MAX_RADIUS by 0.02.
    //   3. If the last few cm creep in slowly, lower FF_MAX_RADIUS by 0.02.
    public static final Distance FF_MIN_RADIUS = Meters.of(0.01);

    public static final Distance FF_MAX_RADIUS = Meters.of(0.05);

    // TODO: ALIGNMENT TOLERANCES
    // How to tune:
    //   1. Measure how far off your mechanism can be and still score. Set TRANSLATION_TOLERANCE
    //      and ROTATION_TOLERANCE to half of that.
    //   2. If aligns time out, plot "DriveToPose/DistanceError" while parked. If it jumps more than
    //      the tolerance, fix vision (VisionConstants) before loosening tolerances.
    //   3. Raise SETTLE_TIME if the robot is still rocking when it scores.
    public static final Distance TRANSLATION_TOLERANCE = Meters.of(0.015);

    public static final Angle ROTATION_TOLERANCE = Degrees.of(1.0);

    public static final LinearVelocity VELOCITY_TOLERANCE = MetersPerSecond.of(0.05);

    public static final AngularVelocity OMEGA_TOLERANCE = RadiansPerSecond.of(0.1);

    public static final Time SETTLE_TIME = Seconds.of(0.1);

    public static final Time TIMEOUT = Seconds.of(3.0);

    // TODO: AIM-AT-TARGET
    // How to tune:
    //   1. Hold the aim button while strafing past the target. Plot "Alignment/AimError".
    //   2. If the robot can't stay pointed at the target, lower AIM_TRANSLATION_SCALE.
    //   3. If the heading lags behind the target while moving, raise AIM_LOOKAHEAD by 0.1 s.
    //      If it leads, lower it.
    //   4. Set AIM_LOCK_TOLERANCE to the largest aim error you can still score from.
    public static final double AIM_TRANSLATION_SCALE = 0.3;

    public static final Time AIM_LOOKAHEAD = Milliseconds.of(500);

    public static final Angle AIM_LOCK_TOLERANCE = Degrees.of(3);
  }

  public static final class Auto {

    private Auto() {}

    // TODO: PATHPLANNER FOLLOWING GAINS
    // How to tune:
    //   1. Finish DriveConstants tuning first.
    //   2. Run a 3 m straight path and a path with a 90 deg turn at full speed.
    //   3. Plot "PathPlanner/targetPose" vs "RobotState/EstimatedPose" and
    // "PathPlanner/inaccuracy".
    //   4. If the robot always lags behind the target, fix drive kV/kA or wheel radius instead.
    //   5. If it weaves, lower P by 1.0. If it corrects too slowly, raise P by 1.0.
    public static final double TRANSLATION_P = 4.0;

    public static final double TRANSLATION_I = 0.0;

    public static final double TRANSLATION_D = 0.0;

    public static final double ROTATION_P = 4.0;

    public static final double ROTATION_I = 0.0;

    public static final double ROTATION_D = 0.0;

    // TODO: PATHFINDING CONSTRAINTS
    // How to tune:
    //   1. Run a "Path ..." named command or the Y button.
    //   2. Raise velocity and acceleration by 0.5 until wheels slip or bumpers hit field elements,
    //      then lower them by 0.5.
    public static final LinearVelocity PATHFIND_MAX_VELOCITY = MetersPerSecond.of(3.0);

    public static final LinearAcceleration PATHFIND_MAX_ACCELERATION =
        MetersPerSecondPerSecond.of(3.0);

    public static final AngularVelocity PATHFIND_MAX_ANGULAR_VELOCITY = DegreesPerSecond.of(540);

    public static final AngularAcceleration PATHFIND_MAX_ANGULAR_ACCELERATION =
        DegreesPerSecondPerSecond.of(720);

    public static final LinearVelocity PATHFIND_GOAL_END_VELOCITY = MetersPerSecond.of(0);

    // TODO: PATH CHAINING DISTANCES
    // How to tune:
    //   1. Run your autos and read "Auto/Chain/ReanchorDistance" at the start of each path.
    //   2. Set CHAIN_MAX_REANCHOR_DISTANCE to the largest normal value plus 0.5 m.
    public static final Distance CHAIN_MIN_REANCHOR_DISTANCE = Meters.of(0.02);

    public static final Distance CHAIN_MAX_REANCHOR_DISTANCE = Meters.of(1.5);

    public static final String DEFAULT_AUTO_NAME = "Do Nothing";
  }

  public static final class Power {

    private Power() {}

    public static final int PDH_ID = 1;

    // TODO: BATTERY ALERTS (volts)
    // How to use:
    //   1. Swap the battery when the "Low Battery" alert shows in the pits.
    //   2. If "Critical Battery" shows during a match, check the battery, breaker and connections.
    public static final double BATTERY_WARNING_VOLTAGE = 12.0;

    public static final double BATTERY_CRITICAL_VOLTAGE = 11.5;

    public static final Time BATTERY_CHECK_INTERVAL = Seconds.of(5);
  }
}
