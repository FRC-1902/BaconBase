package frc.robot.subsystems.drive;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.util.MapleSimUtil;
import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.drivesims.SwerveModuleSimulation;
import org.ironmaple.simulation.motorsims.SimulatedMotorController;

public class ModuleIOSim implements ModuleIO {

  private static final double MAX_VOLTS = 12.0;

  private final SwerveModuleSimulation moduleSimulation;

  private final OnboardController driveController;

  private final OnboardController steerController;

  public ModuleIOSim(SwerveModuleSimulation moduleSimulation) {
    this.moduleSimulation = moduleSimulation;
    driveController =
        moduleSimulation.useDriveMotorController(
            new OnboardController(
                DriveConstants.HARDWARE.driveMotor(),
                DriveConstants.DRIVE_CURRENT_LIMIT,
                DriveConstants.DRIVE_KP,
                DriveConstants.DRIVE_KD,
                false));
    steerController =
        moduleSimulation.useSteerMotorController(
            new OnboardController(
                DriveConstants.HARDWARE.steerMotor(),
                DriveConstants.STEER_CURRENT_LIMIT,
                DriveConstants.STEER_KP,
                DriveConstants.STEER_KD,
                true));
  }

  @Override
  public void updateInputs(ModuleIOInputs inputs) {
    double wheelRadiusMeters = DriveConstants.WHEEL_RADIUS.in(Meters);
    Rotation2d steerFacing = moduleSimulation.getSteerAbsoluteFacing();

    inputs.driveConnected = true;
    inputs.drivePositionMeters =
        moduleSimulation.getDriveWheelFinalPosition().in(Radians) * wheelRadiusMeters;
    inputs.driveVelocityMetersPerSec =
        moduleSimulation.getDriveWheelFinalSpeed().in(RadiansPerSecond) * wheelRadiusMeters;
    inputs.driveAppliedVolts = moduleSimulation.getDriveMotorAppliedVoltage().in(Volts);
    inputs.driveCurrentAmps = Math.abs(moduleSimulation.getDriveMotorStatorCurrent().in(Amps));

    inputs.steerConnected = true;
    inputs.steerPosition = steerFacing;
    inputs.steerVelocityRadPerSec =
        moduleSimulation.getSteerAbsoluteEncoderSpeed().in(RadiansPerSecond);
    inputs.steerAppliedVolts = moduleSimulation.getSteerMotorAppliedVoltage().in(Volts);
    inputs.steerCurrentAmps = Math.abs(moduleSimulation.getSteerMotorStatorCurrent().in(Amps));

    inputs.encoderConnected = true;
    inputs.absolutePosition = steerFacing;
    inputs.absolutePositionRaw = steerFacing;
    inputs.steerResyncCount = 0;

    inputs.odometryTimestamps = MapleSimUtil.getSimulationOdometryTimeStamps();
    Angle[] wheelPositions = moduleSimulation.getCachedDriveWheelFinalPositions();
    inputs.odometryDrivePositionsMeters = new double[wheelPositions.length];
    for (int i = 0; i < wheelPositions.length; i++) {
      inputs.odometryDrivePositionsMeters[i] = wheelPositions[i].in(Radians) * wheelRadiusMeters;
    }
    inputs.odometrySteerPositions = moduleSimulation.getCachedSteerAbsolutePositions();
  }

  @Override
  public void setDriveOpenLoop(double volts) {
    driveController.runOpenLoop(volts);
  }

  @Override
  public void setSteerOpenLoop(double volts) {
    steerController.runOpenLoop(volts);
  }

  @Override
  public void setDriveVelocity(double velocityMetersPerSec, double feedforwardVolts) {
    driveController.runClosedLoop(
        velocityMetersPerSec / DriveConstants.WHEEL_RADIUS.in(Meters), feedforwardVolts);
  }

  @Override
  public void setSteerPosition(Rotation2d position) {
    steerController.runClosedLoop(position.getRadians(), 0.0);
  }

  @Override
  public void setDrivePID(double kP, double kD) {
    double wheelRadiusMeters = DriveConstants.WHEEL_RADIUS.in(Meters);
    driveController.setGains(kP * wheelRadiusMeters, kD * wheelRadiusMeters);
  }

  @Override
  public void setSteerPID(double kP, double kD) {
    steerController.setGains(kP, kD);
  }

  private static final class OnboardController implements SimulatedMotorController {

    private final GenericMotorController currentLimiter;

    private final boolean positionControl;

    private double kP;

    private double kD;

    private boolean closedLoop = false;

    private double openLoopVolts = 0.0;

    private double setpoint = 0.0;

    private double feedforwardVolts = 0.0;

    private double lastError = Double.NaN;

    private OnboardController(
        DCMotor motor, int currentLimitAmps, double kP, double kD, boolean positionControl) {
      this.currentLimiter =
          new GenericMotorController(motor).withCurrentLimit(Amps.of(currentLimitAmps));
      this.positionControl = positionControl;
      double scale = positionControl ? 1.0 : DriveConstants.WHEEL_RADIUS.in(Meters);
      this.kP = kP * scale;
      this.kD = kD * scale;
    }

    private void setGains(double kP, double kD) {
      this.kP = kP;
      this.kD = kD;
    }

    private void runOpenLoop(double volts) {
      closedLoop = false;
      openLoopVolts = volts;
      lastError = Double.NaN;
    }

    private void runClosedLoop(double setpoint, double feedforwardVolts) {
      if (!closedLoop) {
        lastError = Double.NaN;
      }
      closedLoop = true;
      this.setpoint = setpoint;
      this.feedforwardVolts = feedforwardVolts;
    }

    @Override
    public Voltage updateControlSignal(
        Angle mechanismAngle,
        AngularVelocity mechanismVelocity,
        Angle encoderAngle,
        AngularVelocity encoderVelocity) {
      double volts = openLoopVolts;
      if (closedLoop) {
        double error =
            positionControl
                ? MathUtil.angleModulus(setpoint - mechanismAngle.in(Radians))
                : setpoint - mechanismVelocity.in(RadiansPerSecond);
        double dt = SimulatedArena.getSimulationDt().in(Seconds);
        double derivative = Double.isNaN(lastError) ? 0.0 : (error - lastError) / dt;
        lastError = error;
        volts = feedforwardVolts + kP * error + kD * derivative;
      }
      volts = MathUtil.clamp(volts, -MAX_VOLTS, MAX_VOLTS);
      return currentLimiter.constrainOutputVoltage(encoderAngle, encoderVelocity, Volts.of(volts));
    }
  }
}
