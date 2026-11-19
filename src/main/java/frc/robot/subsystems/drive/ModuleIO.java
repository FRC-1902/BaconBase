package frc.robot.subsystems.drive;

import edu.wpi.first.math.geometry.Rotation2d;
import org.littletonrobotics.junction.AutoLog;

public interface ModuleIO {

  @AutoLog
  class ModuleIOInputs {
    public boolean driveConnected = false;
    public double drivePositionMeters = 0.0;
    public double driveVelocityMetersPerSec = 0.0;
    public double driveAppliedVolts = 0.0;
    public double driveCurrentAmps = 0.0;

    public boolean steerConnected = false;
    public Rotation2d steerPosition = Rotation2d.kZero;
    public double steerVelocityRadPerSec = 0.0;
    public double steerAppliedVolts = 0.0;
    public double steerCurrentAmps = 0.0;

    public boolean encoderConnected = false;
    public Rotation2d absolutePosition = Rotation2d.kZero;
    public Rotation2d absolutePositionRaw = Rotation2d.kZero;
    public int steerResyncCount = 0;

    public double[] odometryTimestamps = new double[] {};
    public double[] odometryDrivePositionsMeters = new double[] {};
    public Rotation2d[] odometrySteerPositions = new Rotation2d[] {};
  }

  default void updateInputs(ModuleIOInputs inputs) {}

  default void setDriveOpenLoop(double volts) {}

  default void setSteerOpenLoop(double volts) {}

  default void setDriveVelocity(double velocityMetersPerSec, double feedforwardVolts) {}

  default void setSteerPosition(Rotation2d position) {}

  default void setDrivePID(double kP, double kD) {}

  default void setSteerPID(double kP, double kD) {}

  default void setDriveBrakeMode(boolean enabled) {}
}
