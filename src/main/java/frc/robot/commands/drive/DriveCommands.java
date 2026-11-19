package frc.robot.commands.drive;

import static edu.wpi.first.units.Units.Seconds;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants.Operator;
import frc.robot.RobotState;
import frc.robot.field.AllianceFlipUtil;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.DriveConstants;
import frc.robot.util.LoggedTunableNumber;
import java.text.DecimalFormat;
import java.text.NumberFormat;
import java.util.ArrayList;
import java.util.List;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.Logger;

public final class DriveCommands {

  private static final LoggedTunableNumber HEADING_HOLD_KP =
      new LoggedTunableNumber("Teleop/HeadingHoldKp", Operator.HEADING_HOLD_KP);

  private static final LoggedTunableNumber HEADING_HOLD_DELAY_SECONDS =
      new LoggedTunableNumber(
          "Teleop/HeadingHoldDelaySeconds", Operator.HEADING_HOLD_DELAY.in(Seconds));

  private static final double HEADING_HOLD_RELATCH_RADIANS = Math.toRadians(30.0);

  private DriveCommands() {}

  public static double applyInputCurve(double magnitude) {
    return Operator.INPUT_LINEAR_WEIGHT * magnitude
        + Operator.INPUT_CUBIC_WEIGHT * magnitude * magnitude * magnitude;
  }

  public static Translation2d shapeTranslation(double forward, double left) {
    double magnitude = Math.hypot(forward, left);
    if (magnitude <= Operator.TRANSLATION_DEADBAND) {
      return Translation2d.kZero;
    }
    double scaled =
        Math.min(
            1.0,
            (magnitude - Operator.TRANSLATION_DEADBAND) / (1.0 - Operator.TRANSLATION_DEADBAND));
    double shaped = applyInputCurve(scaled);
    return new Translation2d(shaped * forward / magnitude, shaped * left / magnitude);
  }

  public static double shapeRotation(double rotation) {
    double deadbanded = MathUtil.applyDeadband(rotation, Operator.ROTATION_DEADBAND);
    return Math.copySign(applyInputCurve(Math.abs(deadbanded)), deadbanded);
  }

  public static Translation2d fieldTranslationVelocity(
      double forward, double left, double maxSpeedMetersPerSec) {
    Translation2d velocity = shapeTranslation(forward, left).times(maxSpeedMetersPerSec);
    return AllianceFlipUtil.shouldFlip() ? velocity.rotateBy(Rotation2d.kPi) : velocity;
  }

  public static Command joystickDrive(
      Drive drive, DoubleSupplier forward, DoubleSupplier left, DoubleSupplier rotation) {
    HeadingHoldState state = new HeadingHoldState();
    return Commands.run(
            () -> {
              Translation2d linearVelocity =
                  fieldTranslationVelocity(
                      forward.getAsDouble(),
                      left.getAsDouble(),
                      DriveConstants.MAX_LINEAR_SPEED * Operator.TRANSLATION_SCALE);
              Rotation2d heading = RobotState.getInstance().getRotation();
              double omega = state.calculate(shapeRotation(rotation.getAsDouble()), heading);
              drive.runVelocity(
                  ChassisSpeeds.fromFieldRelativeSpeeds(
                      linearVelocity.getX(), linearVelocity.getY(), omega, heading));
            },
            drive)
        .beforeStarting(state::reset)
        .withName("JoystickDrive");
  }

  public static Command feedforwardCharacterization(Drive drive) {
    List<Double> velocitySamples = new ArrayList<>();
    List<Double> voltageSamples = new ArrayList<>();
    Timer timer = new Timer();

    return Commands.sequence(
            Commands.runOnce(
                () -> {
                  velocitySamples.clear();
                  voltageSamples.clear();
                }),
            Commands.run(() -> drive.runCharacterization(0.0), drive)
                .withTimeout(DriveConstants.FF_START_DELAY),
            Commands.runOnce(timer::restart),
            Commands.run(
                    () -> {
                      double voltage = timer.get() * DriveConstants.FF_RAMP_RATE;
                      drive.runCharacterization(voltage);
                      velocitySamples.add(drive.getFFCharacterizationVelocity());
                      voltageSamples.add(voltage);
                      Logger.recordOutput("Drive/Characterization/FF/Voltage", voltage);
                    },
                    drive)
                .finallyDo(() -> reportFeedforward(velocitySamples, voltageSamples)))
        .withName("DriveFeedforwardCharacterization");
  }

  public static Command wheelRadiusCharacterization(Drive drive) {
    SlewRateLimiter limiter = new SlewRateLimiter(DriveConstants.WHEEL_RADIUS_RAMP_RATE);
    WheelRadiusState state = new WheelRadiusState();

    return Commands.parallel(
            Commands.sequence(
                Commands.runOnce(() -> limiter.reset(0.0)),
                Commands.run(
                    () ->
                        drive.runVelocity(
                            new ChassisSpeeds(
                                0.0,
                                0.0,
                                limiter.calculate(DriveConstants.WHEEL_RADIUS_MAX_VELOCITY))),
                    drive)),
            Commands.sequence(
                Commands.waitSeconds(1.0),
                Commands.runOnce(
                    () -> {
                      state.positions = drive.getWheelRadiusCharacterizationPositions();
                      state.lastAngle = gyroRotation();
                      state.gyroDelta = 0.0;
                    }),
                Commands.run(
                        () -> {
                          Rotation2d rotation = gyroRotation();
                          state.gyroDelta += Math.abs(rotation.minus(state.lastAngle).getRadians());
                          state.lastAngle = rotation;
                        })
                    .finallyDo(() -> reportWheelRadius(drive, state))))
        .withName("DriveWheelRadiusCharacterization");
  }

  private static Rotation2d gyroRotation() {
    return RobotState.getInstance().getOdometryPose().getRotation();
  }

  private static void reportFeedforward(List<Double> velocities, List<Double> voltages) {
    int n = velocities.size();
    double sumX = 0.0;
    double sumY = 0.0;
    double sumXY = 0.0;
    double sumX2 = 0.0;
    for (int i = 0; i < n; i++) {
      sumX += velocities.get(i);
      sumY += voltages.get(i);
      sumXY += velocities.get(i) * voltages.get(i);
      sumX2 += velocities.get(i) * velocities.get(i);
    }
    double denominator = n * sumX2 - sumX * sumX;
    if (n < 2 || Math.abs(denominator) < 1e-9) {
      System.out.println("********** Drive FF Characterization: not enough data **********");
      return;
    }
    double kS = (sumY * sumX2 - sumX * sumXY) / denominator;
    double kV = (n * sumXY - sumX * sumY) / denominator;

    NumberFormat formatter = new DecimalFormat("#0.00000");
    System.out.println("********** Drive FF Characterization Results **********");
    System.out.println("\tkS: " + formatter.format(kS) + " V");
    System.out.println("\tkV: " + formatter.format(kV) + " V/(m/s)");
    Logger.recordOutput("Drive/Characterization/FF/kS", kS);
    Logger.recordOutput("Drive/Characterization/FF/kV", kV);
    Logger.recordOutput("Drive/Characterization/FF/SampleCount", n);
  }

  private static void reportWheelRadius(Drive drive, WheelRadiusState state) {
    double[] positions = drive.getWheelRadiusCharacterizationPositions();
    int count = Math.min(positions.length, state.positions.length);
    double wheelDelta = 0.0;
    for (int i = 0; i < count; i++) {
      wheelDelta += Math.abs(positions[i] - state.positions[i]) / count;
    }
    if (count == 0 || wheelDelta < 1e-9) {
      System.out.println("********** Wheel Radius Characterization: not enough data **********");
      return;
    }
    double wheelRadius = (state.gyroDelta * DriveConstants.DRIVE_BASE_RADIUS) / wheelDelta;

    NumberFormat formatter = new DecimalFormat("#0.000");
    System.out.println("********** Wheel Radius Characterization Results **********");
    System.out.println("\tWheel Delta: " + formatter.format(wheelDelta) + " radians");
    System.out.println("\tGyro Delta: " + formatter.format(state.gyroDelta) + " radians");
    System.out.println(
        "\tWheel Radius: "
            + formatter.format(wheelRadius)
            + " meters, "
            + formatter.format(Units.metersToInches(wheelRadius))
            + " inches");
    Logger.recordOutput("Drive/Characterization/WheelRadius/WheelDeltaRadians", wheelDelta);
    Logger.recordOutput("Drive/Characterization/WheelRadius/GyroDeltaRadians", state.gyroDelta);
    Logger.recordOutput("Drive/Characterization/WheelRadius/RadiusMeters", wheelRadius);
    Logger.recordOutput(
        "Drive/Characterization/WheelRadius/RadiusInches", Units.metersToInches(wheelRadius));
  }

  private static final class HeadingHoldState {

    private Rotation2d target;

    private double lastRotationInputTimestamp;

    void reset() {
      target = null;
      lastRotationInputTimestamp = Timer.getTimestamp();
    }

    double calculate(double rotationInput, Rotation2d heading) {
      double now = Timer.getTimestamp();
      double omega;
      if (!Operator.HEADING_HOLD_ENABLED || rotationInput != 0.0) {
        target = null;
        lastRotationInputTimestamp = now;
        omega = rotationInput * DriveConstants.MAX_ANGULAR_SPEED * Operator.ROTATION_SCALE;
      } else {
        if (target == null
            && now - lastRotationInputTimestamp >= HEADING_HOLD_DELAY_SECONDS.get()) {
          target = heading;
        }
        if (target != null
            && Math.abs(target.minus(heading).getRadians()) > HEADING_HOLD_RELATCH_RADIANS) {
          target = heading;
        }
        omega = target == null ? 0.0 : HEADING_HOLD_KP.get() * target.minus(heading).getRadians();
      }
      Logger.recordOutput("Teleop/HeadingHoldActive", target != null);
      Logger.recordOutput("Teleop/HeadingHoldTarget", target == null ? heading : target);
      return omega;
    }
  }

  private static final class WheelRadiusState {

    private double[] positions = new double[0];

    private Rotation2d lastAngle = Rotation2d.kZero;

    private double gyroDelta = 0.0;
  }
}
