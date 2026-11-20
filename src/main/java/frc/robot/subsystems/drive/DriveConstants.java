package frc.robot.subsystems.drive;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.KilogramSquareMeters;
import static edu.wpi.first.units.Units.Kilograms;
import static edu.wpi.first.units.Units.Meters;

import com.ctre.phoenix6.CANBus;
import com.pathplanner.lib.config.ModuleConfig;
import com.pathplanner.lib.config.RobotConfig;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Mass;
import edu.wpi.first.units.measure.MomentOfInertia;

// TODO: DRIVETRAIN TUNING ORDER (fresh battery, on carpet, do the steps in order)
//   1. CAN IDs, encoder offsets, inversions, gyro direction (MODULES)
//   2. Steer kP (STEER_KP)
//   3. Drive kS / kV (DRIVE_KS, DRIVE_KV)
//   4. Drive kA (DRIVE_KA)
//   5. Drive kP (DRIVE_KP)
//   6. Wheel radius (WHEEL_RADIUS)
//   7. Coupling ratio (HARDWARE couplingRatio)
//   8. Mass, MOI, wheel COF (ROBOT_MASS, ROBOT_MOI, WHEEL_COF)
//   9. Max speeds (MAX_LINEAR_SPEED, MAX_STEER_VELOCITY)
//   10. Odometry trust (ODOMETRY_STD_DEVS), then VisionConstants
//   11. PathPlanner gains, then auto-align, then driver feel (Constants.java)

public final class DriveConstants {

  private DriveConstants() {}

  public enum MotorControllerType {
    SPARK_MAX,
    SPARK_FLEX
  }

  public record SwerveModuleHardware(
      String name,
      DCMotor driveMotor,
      MotorControllerType driveController,
      DCMotor steerMotor,
      MotorControllerType steerController,
      double driveRatio,
      double steerRatio,
      double couplingRatio,
      Distance wheelRadius,
      MomentOfInertia steerInertia,
      boolean driveInverted,
      boolean steerInverted,
      boolean encoderClockwisePositive) {}

  public record ModuleSetup(
      String name,
      int driveId,
      int steerId,
      int encoderId,
      Rotation2d encoderOffset,
      Translation2d location) {}

  // TODO: MODULE HARDWARE PRESET
  // Set HARDWARE below to the preset that matches the robot, or add a new preset with the
  // drive ratio, steer ratio and wheel radius.
  // How to measure couplingRatio:
  //   1. Robot on blocks, disabled. Plot "Drive/Module/FrontLeft/DrivePositionMeters".
  //   2. Hold the wheel still and turn the module 5 full turns counterclockwise by hand.
  //   3. If the drive position changed, set couplingRatio += (change * driveRatio)
  //      / (2 * PI * wheelRadiusMeters * 5). If it got worse, subtract instead.
  //   4. Repeat until 5 turns changes the drive position by less than 1 mm.
  // How to check encoderClockwisePositive:
  //   1. Robot disabled, turn a module counterclockwise by hand.
  //   2. "Drive/Module/<Name>/AbsolutePosition" must increase. If it decreases, set true.

  public static final SwerveModuleHardware MK4I_L2_NEO_SPARKMAX =
      new SwerveModuleHardware(
          "MK4i L2 NEO SparkMax",
          DCMotor.getNEO(1),
          MotorControllerType.SPARK_MAX,
          DCMotor.getNEO(1),
          MotorControllerType.SPARK_MAX,
          (50.0 / 14.0) * (17.0 / 27.0) * (45.0 / 15.0),
          150.0 / 7.0,
          0.0,
          Inches.of(2.0),
          KilogramSquareMeters.of(0.03),
          true,
          true,
          false);

  public static final SwerveModuleHardware MK5N_R1_VORTEX_SPARKFLEX =
      mk5nVortexSparkFlex("MK5n R1 Vortex SparkFlex", 7.03);

  public static final SwerveModuleHardware MK5N_R2_VORTEX_SPARKFLEX =
      mk5nVortexSparkFlex("MK5n R2 Vortex SparkFlex", 6.03);

  public static final SwerveModuleHardware MK5N_R3_VORTEX_SPARKFLEX =
      mk5nVortexSparkFlex("MK5n R3 Vortex SparkFlex", 5.27);

  public static final SwerveModuleHardware HARDWARE = MK4I_L2_NEO_SPARKMAX;

  // TODO: CAN IDS, ENCODER OFFSETS, MODULE LOCATIONS
  // How to set up:
  //   1. Set the CAN IDs (REV Hardware Client, or Phoenix Tuner X for CANcoders/Pigeon).
  //   2. Measure wheel center to wheel center, divide by 2, put it in the locations (+X forward,
  //      +Y left). Copy the same numbers into the PathPlanner GUI robot config.
  // How to set encoder offsets:
  //   1. Set all encoderOffset values to Rotation2d.kZero and deploy. Make sure if you are using
  //      CANcoders that their boot setting in Phoenix Tuner X is set to absolute and not boot to
  //      zero.
  //   2. Robot on blocks, disabled. Turn every module so all bevel gears face the robot's LEFT and
  //      the wheels point straight forward (use a straightedge across the wheels).
  //   3. Read "Drive/Module/<Name>/AbsolutePositionRaw" for each module.
  //   4. In MODULES below, replace that module's Rotation2d.kZero with
  //      Rotation2d.fromRadians(<that reading>) and deploy.
  //   5. With the modules straight, "Drive/Module/<Name>/AbsolutePosition" must read 0 +/- 0.01.
  // How to check inversions:
  //   1. Robot on blocks, enable teleop, push the left stick forward.
  //   2. If all wheels spin backwards, flip driveInverted. If only some do, redo that offset.
  //   3. If modules oscillate or spin nonstop, flip steerInverted.
  //   4. On the ground, spin the robot counterclockwise: "RobotState/EstimatedPose" rotation must
  //      increase. If not, fix the Pigeon mount pose in Phoenix Tuner X.
  public static final ModuleSetup[] MODULES = {
    new ModuleSetup(
        "FrontLeft",
        9,
        10,
        43,
        Rotation2d.fromRadians(1.181),
        new Translation2d(Inches.of(12), Inches.of(12))),
    new ModuleSetup(
        "FrontRight",
        6,
        5,
        40,
        Rotation2d.fromRadians(2.203),
        new Translation2d(Inches.of(12), Inches.of(-12))),
    new ModuleSetup(
        "BackLeft",
        1,
        2,
        42,
        Rotation2d.fromRadians(0.084),
        new Translation2d(Inches.of(-12), Inches.of(12))),
    new ModuleSetup(
        "BackRight",
        11,
        4,
        46,
        Rotation2d.fromRadians(-1.416),
        new Translation2d(Inches.of(-12), Inches.of(-12)))
  };

  public static final int PIGEON_ID = 21;

  public static final CANBus CAN_BUS = new CANBus("rio");

  // TODO: WHEEL RADIUS
  // How to measure:
  //   1. Robot on carpet with 1 m of space around it.
  //   2. Auto Chooser -> "Drive Wheel Radius Characterization", enable autonomous.
  //   3. Disable after a few rotations. In AdvantageScope read
  //      "Drive/Characterization/WheelRadius/RadiusMeters".
  //   4. Run 3 times, average, set WHEEL_RADIUS = Meters.of(<average>).
  //   5. Re-measure every few comps and after re-treading.
  public static final Distance WHEEL_RADIUS = HARDWARE.wheelRadius();

  // TODO: ROBOT MASS, MOI, WHEEL COF
  // How to set:
  //   1. ROBOT_MASS: weigh the robot with bumpers and battery (kg = lb * 0.4536).
  //   2. ROBOT_MOI: CAD -> mass properties of the whole robot -> moment of inertia about the
  //      vertical axis, in kg*m^2.
  //   3. WHEEL_COF: leave at 1.19 for SDS billet wheels with nitrile tread. Lower it if the wheels
  //      spin when accelerating hard.
  public static final Mass ROBOT_MASS = Kilograms.of(56.7);

  public static final double ROBOT_MOI = 6.883;

  public static final double WHEEL_COF = 1.15;

  public static final Distance BUMPER_LENGTH = Meters.of(0.876);

  public static final Distance BUMPER_WIDTH = Meters.of(0.864);

  // TODO: DRIVE kS / kV
  // How to measure:
  //   1. Robot on carpet with 4 m of open space in front of it.
  //   2. Auto Chooser -> "Drive Simple FF Characterization", enable autonomous.
  //   3. Disable before it hits anything. In AdvantageScope read "Drive/Characterization/FF/kS"
  //      and "Drive/Characterization/FF/kV".
  //   4. Run 3 times, average, put the results here.
  public static final double DRIVE_KS = theoreticalDriveKs();

  public static final double DRIVE_KV = theoreticalDriveKv();

  // TODO: DRIVE kA
  // How to measure:
  //   1. Run all four "Drive SysId" routines from the Auto Chooser.
  //   2. Load the log into WPILib SysId using the "Drive/Module/FrontLeft/..." signals, "Simple".
  //   3. Put SysId's kA here.
  public static final double DRIVE_KA = theoreticalDriveKa();

  // TODO: DRIVE kP / kD
  // How to tune:
  //   1. Plot "Drive/Module/FrontLeft/DriveVelocityMetersPerSec" against the speed in
  //      "Drive/ModuleStates/Setpoints".
  //   2. Drive back and forth hard.
  //   3. If the motors buzz or the velocity oscillates, halve kP.
  //   4. If the velocity sags while pushing another robot, raise kP by 1.0.
  //   5. Leave kD at 0.
  //   Starting value is the 2025 robot's tuned YAGSL drive p (0.05 duty per m/s * 12 V).
  public static final double DRIVE_KP = 0.6;

  public static final double DRIVE_KD = 0.0;

  // TODO: STEER kP / kD
  // How to tune:
  //   1. Robot on blocks. Plot "Drive/Module/FrontLeft/SteerPosition" against the angle in
  //      "Drive/ModuleStates/Setpoints".
  //   2. Flick the left stick in circles and left/right.
  //   3. Raise kP by 20% until the wheel wobbles before settling, then lower it by 20%.
  //   4. If it still wobbles, add kD in steps of 0.05. Lower it if the module buzzes.
  //   Starting value is the 2025 robot's tuned YAGSL steer p (0.01 duty per degree * 12 V).
  public static final double STEER_KP = 0.01 * 12.0 * 180.0 / Math.PI;

  public static final double STEER_KD = 0.0;

  // TODO: MAX SPEEDS
  // How to measure:
  //   1. MAX_LINEAR_SPEED: drive full stick straight for 3+ m on a fresh battery. Read the peak of
  //      "Drive/ChassisSpeeds/Measured". Set it to 95% of that.
  //   2. MAX_STEER_VELOCITY: robot on blocks, flick the stick full left/right. Read the peak of
  //      "Drive/Module/FrontLeft/SteerVelocityRadPerSec". Set it to 90% of that.
  public static final double MAX_LINEAR_SPEED = (11.0 - DRIVE_KS) / DRIVE_KV;

  public static final double DRIVE_BASE_RADIUS = maxModuleDistance();

  public static final double MAX_ANGULAR_SPEED = MAX_LINEAR_SPEED / DRIVE_BASE_RADIUS;

  public static final double MAX_STEER_VELOCITY =
      HARDWARE.steerMotor().freeSpeedRadPerSec / HARDWARE.steerRatio() * (11.0 / 12.0);

  // TODO: CURRENT LIMITS (amps)
  // How to tune:
  //   1. Leave DRIVE_CURRENT_LIMIT at the calculated wheel-slip current.
  //   2. If the main breaker trips or the robot browns out, lower it to 45.
  //   3. Raise STEER_CURRENT_LIMIT to 30 only if steering is weak when pushed.
  public static final int DRIVE_CURRENT_LIMIT = theoreticalSlipCurrentLimit();

  public static final int STEER_CURRENT_LIMIT = 20;

  // TODO: ODOMETRY SAMPLE RATE (Hz)
  // How to tune:
  //   1. Leave at 100.
  //   2. Only raise it if "/SystemStats/CANBus/Utilization" stays under 70%.
  public static final double ODOMETRY_FREQUENCY_HZ = 100.0;

  // TODO: STEER ENCODER RE-SYNC (only runs while disabled)
  // How to tune:
  //   1. Drive around, then watch "Drive/Module/<Name>/SteerEncoderErrorDegrees".
  //   2. A healthy MK4i stays within a few degrees (gear backlash).
  //   3. If one module drifts far from 0, check that module's CANcoder ID, magnet and steer gear.
  public static final double STEER_RESYNC_THRESHOLD_RADIANS = Math.toRadians(1.0);

  public static final double STEER_RESYNC_MAX_VELOCITY_RAD_PER_SEC = 0.05;

  public static final int STEER_RESYNC_CYCLES = 25;

  // TODO: ODOMETRY TRUST
  // How to tune:
  //   1. Tune vision first. Drive around the field.
  //   2. Plot "RobotState/EstimatedPose" and "RobotState/OdometryPose".
  //   3. If the pose jumps when tags come into view, lower these to 0.05.
  //   4. If drift after collisions is corrected too slowly, raise these to 0.2.
  public static final double[] ODOMETRY_STD_DEVS = {0.1, 0.1, 0.1};

  public static final double FF_RAMP_RATE = 0.1;

  public static final double FF_START_DELAY = 2.0;

  public static final double WHEEL_RADIUS_MAX_VELOCITY = 0.25;

  public static final double WHEEL_RADIUS_RAMP_RATE = 0.05;

  public static final Translation2d[] MODULE_TRANSLATIONS = {
    MODULES[0].location(), MODULES[1].location(), MODULES[2].location(), MODULES[3].location()
  };

  public static final SwerveDriveKinematics KINEMATICS =
      new SwerveDriveKinematics(MODULE_TRANSLATIONS);

  public static final RobotConfig PATHPLANNER_CONFIG =
      new RobotConfig(
          ROBOT_MASS.in(Kilograms),
          ROBOT_MOI,
          new ModuleConfig(
              WHEEL_RADIUS.in(Meters),
              MAX_LINEAR_SPEED,
              WHEEL_COF,
              HARDWARE.driveMotor().withReduction(HARDWARE.driveRatio()),
              DRIVE_CURRENT_LIMIT,
              1),
          MODULE_TRANSLATIONS);

  private static SwerveModuleHardware mk5nVortexSparkFlex(String name, double driveRatio) {
    return new SwerveModuleHardware(
        name,
        DCMotor.getNeoVortex(1),
        MotorControllerType.SPARK_FLEX,
        DCMotor.getNeoVortex(1),
        MotorControllerType.SPARK_FLEX,
        driveRatio,
        287.0 / 11.0,
        0.0,
        Inches.of(2.0),
        KilogramSquareMeters.of(0.03),
        false,
        true,
        false);
  }

  private static double wheelRadiusMeters() {
    return WHEEL_RADIUS.in(Meters);
  }

  private static double theoreticalDriveKs() {
    DCMotor motor = HARDWARE.driveMotor();
    return motor.freeCurrentAmps * motor.rOhms;
  }

  private static double theoreticalDriveKv() {
    return HARDWARE.driveRatio() / (wheelRadiusMeters() * HARDWARE.driveMotor().KvRadPerSecPerVolt);
  }

  private static double theoreticalDriveKa() {
    DCMotor motor = HARDWARE.driveMotor();
    return (ROBOT_MASS.in(Kilograms) / 4.0)
        * wheelRadiusMeters()
        * motor.rOhms
        / (HARDWARE.driveRatio() * motor.KtNMPerAmp);
  }

  private static int theoreticalSlipCurrentLimit() {
    double wheelForce = WHEEL_COF * ROBOT_MASS.in(Kilograms) * 9.81 / 4.0;
    double motorTorque = wheelForce * wheelRadiusMeters() / HARDWARE.driveRatio();
    double slipCurrent = motorTorque / HARDWARE.driveMotor().KtNMPerAmp;
    return (int) Math.min(80.0, Math.floor(slipCurrent));
  }

  private static double maxModuleDistance() {
    double max = 0.0;
    for (ModuleSetup module : MODULES) {
      max = Math.max(max, module.location().getNorm());
    }
    return max;
  }
}
