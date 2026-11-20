package frc.robot.subsystems.drive;

import static edu.wpi.first.units.Units.Meters;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import frc.robot.util.LoggedTunableNumber;
import org.littletonrobotics.junction.Logger;

public class Module {

  private static final double KS_RAMP_METERS_PER_SEC = 0.05;

  private static final double STEER_ENCODER_MISMATCH_DEGREES = 5.0;

  private static final LoggedTunableNumber DRIVE_KS =
      new LoggedTunableNumber("Drive/DriveKs", DriveConstants.DRIVE_KS);

  private static final LoggedTunableNumber DRIVE_KV =
      new LoggedTunableNumber("Drive/DriveKv", DriveConstants.DRIVE_KV);

  private static final LoggedTunableNumber DRIVE_KA =
      new LoggedTunableNumber("Drive/DriveKa", DriveConstants.DRIVE_KA);

  private static final LoggedTunableNumber DRIVE_KP =
      new LoggedTunableNumber("Drive/DriveKp", DriveConstants.DRIVE_KP);

  private static final LoggedTunableNumber DRIVE_KD =
      new LoggedTunableNumber("Drive/DriveKd", DriveConstants.DRIVE_KD);

  private static final LoggedTunableNumber STEER_KP =
      new LoggedTunableNumber("Drive/SteerKp", DriveConstants.STEER_KP);

  private static final LoggedTunableNumber STEER_KD =
      new LoggedTunableNumber("Drive/SteerKd", DriveConstants.STEER_KD);

  private final ModuleIO io;

  private final ModuleIOInputsAutoLogged inputs = new ModuleIOInputsAutoLogged();

  private final String name;

  private final Alert driveDisconnectedAlert;

  private final Alert steerDisconnectedAlert;

  private final Alert encoderDisconnectedAlert;

  private final Alert steerEncoderMismatchAlert;

  private SwerveModulePosition[] odometryPositions = new SwerveModulePosition[] {};

  public Module(ModuleIO io, String name) {
    this.io = io;
    this.name = name;
    driveDisconnectedAlert =
        new Alert("Disconnected drive motor on " + name + " module.", AlertType.kError);
    steerDisconnectedAlert =
        new Alert("Disconnected steer motor on " + name + " module.", AlertType.kError);
    encoderDisconnectedAlert =
        new Alert("Disconnected CANcoder on " + name + " module.", AlertType.kError);
    steerEncoderMismatchAlert =
        new Alert(
            "Steer motor encoder and CANcoder disagree on "
                + name
                + " module. Check its CANcoder ID, direction and magnet.",
            AlertType.kWarning);
  }

  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Drive/Module/" + name, inputs);

    LoggedTunableNumber.ifChanged(
        hashCode(), values -> io.setDrivePID(values[0], values[1]), DRIVE_KP, DRIVE_KD);
    LoggedTunableNumber.ifChanged(
        hashCode(), values -> io.setSteerPID(values[0], values[1]), STEER_KP, STEER_KD);

    int sampleCount =
        Math.min(inputs.odometryTimestamps.length, inputs.odometryDrivePositionsMeters.length);
    sampleCount = Math.min(sampleCount, inputs.odometrySteerPositions.length);
    odometryPositions = new SwerveModulePosition[sampleCount];
    for (int i = 0; i < sampleCount; i++) {
      odometryPositions[i] =
          new SwerveModulePosition(
              inputs.odometryDrivePositionsMeters[i], inputs.odometrySteerPositions[i]);
    }

    driveDisconnectedAlert.set(!inputs.driveConnected);
    steerDisconnectedAlert.set(!inputs.steerConnected);
    encoderDisconnectedAlert.set(!inputs.encoderConnected);

    double steerEncoderErrorDegrees =
        Math.toDegrees(
            MathUtil.angleModulus(
                inputs.steerPosition.minus(inputs.absolutePosition).getRadians()));
    Logger.recordOutput(
        "Drive/Module/" + name + "/SteerEncoderErrorDegrees", steerEncoderErrorDegrees);
    steerEncoderMismatchAlert.set(
        inputs.encoderConnected
            && Math.abs(steerEncoderErrorDegrees) > STEER_ENCODER_MISMATCH_DEGREES);
  }

  public void runSetpoint(SwerveModuleState state, double accelerationMetersPerSecSq) {
    double cosineScale = state.angle.minus(getAngle()).getCos();
    double velocity = state.speedMetersPerSecond * cosineScale;
    double acceleration = accelerationMetersPerSecSq * cosineScale;
    double feedforwardVolts =
        DRIVE_KS.get() * MathUtil.clamp(velocity / KS_RAMP_METERS_PER_SEC, -1.0, 1.0)
            + DRIVE_KV.get() * velocity
            + DRIVE_KA.get() * acceleration;
    io.setDriveVelocity(velocity, feedforwardVolts);
    io.setSteerPosition(state.angle);
  }

  public void runCharacterization(double volts) {
    io.setDriveOpenLoop(volts);
    io.setSteerPosition(Rotation2d.kZero);
  }

  public void stop() {
    io.setDriveOpenLoop(0.0);
    io.setSteerOpenLoop(0.0);
  }

  public void setDriveBrakeMode(boolean enabled) {
    io.setDriveBrakeMode(enabled);
  }

  public Rotation2d getAngle() {
    return inputs.steerPosition;
  }

  public double getPositionMeters() {
    return inputs.drivePositionMeters;
  }

  public double getVelocityMetersPerSec() {
    return inputs.driveVelocityMetersPerSec;
  }

  public SwerveModulePosition getPosition() {
    return new SwerveModulePosition(getPositionMeters(), getAngle());
  }

  public SwerveModuleState getState() {
    return new SwerveModuleState(getVelocityMetersPerSec(), getAngle());
  }

  public SwerveModulePosition[] getOdometryPositions() {
    return odometryPositions;
  }

  public double[] getOdometryTimestamps() {
    return inputs.odometryTimestamps;
  }

  public double getWheelRadiusCharacterizationPosition() {
    return inputs.drivePositionMeters / DriveConstants.WHEEL_RADIUS.in(Meters);
  }

  public double getFFCharacterizationVelocity() {
    return inputs.driveVelocityMetersPerSec;
  }
}
