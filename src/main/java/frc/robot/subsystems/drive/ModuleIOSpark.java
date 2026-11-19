package frc.robot.subsystems.drive;

import static edu.wpi.first.units.Units.Meters;
import static frc.robot.util.SparkUtil.ifOk;
import static frc.robot.util.SparkUtil.tryUntilOk;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.revrobotics.PersistMode;
import com.revrobotics.REVLibError;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkClosedLoopController.ArbFFUnits;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.Angle;
import frc.robot.util.PhoenixUtil;
import frc.robot.util.SparkUtil;
import java.util.Queue;
import java.util.function.DoubleSupplier;

public class ModuleIOSpark implements ModuleIO {

  private static final double NOMINAL_VOLTAGE = 12.0;

  private static final int CONFIG_ATTEMPTS = 5;

  private static final double CONFIG_TIMEOUT_SECONDS = 0.25;

  private static final double CANCODER_TIMEOUT_SECONDS = 0.1;

  private static final double CONNECTED_DEBOUNCE_SECONDS = 0.5;

  private static final double ENCODER_UPDATE_FREQUENCY_HZ = 50.0;

  private static final int STATUS_PERIOD_MS = 20;

  private static final double SPARK_KD_MILLISECONDS_PER_SECOND = 1000.0;

  private final DriveConstants.MotorControllerType driveControllerType;

  private final DriveConstants.MotorControllerType steerControllerType;

  private final Rotation2d encoderOffset;

  private final double couplingMetersPerSteerRadian;

  private final SparkBase driveSpark;

  private final SparkBase steerSpark;

  private final RelativeEncoder driveEncoder;

  private final RelativeEncoder steerEncoder;

  private final SparkClosedLoopController driveController;

  private final SparkClosedLoopController steerController;

  private final CANcoder cancoder;

  private final StatusSignal<Angle> absolutePositionSignal;

  private final Queue<Double> timestampQueue;

  private final Queue<Double> drivePositionQueue;

  private final Queue<Double> steerPositionQueue;

  private final Debouncer driveConnectedDebounce =
      new Debouncer(CONNECTED_DEBOUNCE_SECONDS, Debouncer.DebounceType.kFalling);

  private final Debouncer steerConnectedDebounce =
      new Debouncer(CONNECTED_DEBOUNCE_SECONDS, Debouncer.DebounceType.kFalling);

  private final Debouncer encoderConnectedDebounce =
      new Debouncer(CONNECTED_DEBOUNCE_SECONDS, Debouncer.DebounceType.kFalling);

  private double steerCouplingOriginRad = 0.0;

  private double driveRawPositionMeters = 0.0;

  private double driveRawVelocityMetersPerSec = 0.0;

  private double steerPositionRad = 0.0;

  private double steerVelocityRadPerSec = 0.0;

  private Rotation2d absolutePositionRaw = Rotation2d.kZero;

  private Rotation2d absolutePosition = Rotation2d.kZero;

  private int steerResyncCycles = 0;

  private int steerResyncCount = 0;

  public ModuleIOSpark(DriveConstants.ModuleSetup setup) {
    DriveConstants.SwerveModuleHardware hardware = DriveConstants.HARDWARE;
    double wheelRadiusMeters = DriveConstants.WHEEL_RADIUS.in(Meters);
    driveControllerType = hardware.driveController();
    steerControllerType = hardware.steerController();
    encoderOffset = setup.encoderOffset();
    couplingMetersPerSteerRadian =
        hardware.couplingRatio() * wheelRadiusMeters / hardware.driveRatio();

    driveSpark = createSpark(setup.driveId(), driveControllerType);
    steerSpark = createSpark(setup.steerId(), steerControllerType);
    driveEncoder = driveSpark.getEncoder();
    steerEncoder = steerSpark.getEncoder();
    driveController = driveSpark.getClosedLoopController();
    steerController = steerSpark.getClosedLoopController();

    cancoder = new CANcoder(setup.encoderId(), DriveConstants.CAN_BUS);
    CANcoderConfiguration cancoderConfig = new CANcoderConfiguration();
    PhoenixUtil.tryUntilOk(
        CONFIG_ATTEMPTS,
        () -> cancoder.getConfigurator().refresh(cancoderConfig, CONFIG_TIMEOUT_SECONDS));
    cancoderConfig.MagnetSensor.SensorDirection =
        hardware.encoderClockwisePositive()
            ? SensorDirectionValue.Clockwise_Positive
            : SensorDirectionValue.CounterClockwise_Positive;
    cancoderConfig.MagnetSensor.AbsoluteSensorDiscontinuityPoint = 0.5;
    PhoenixUtil.tryUntilOk(
        CONFIG_ATTEMPTS,
        () -> cancoder.getConfigurator().apply(cancoderConfig, CONFIG_TIMEOUT_SECONDS));
    absolutePositionSignal = cancoder.getAbsolutePosition();
    PhoenixUtil.tryUntilOk(
        CONFIG_ATTEMPTS,
        () -> absolutePositionSignal.setUpdateFrequency(ENCODER_UPDATE_FREQUENCY_HZ));
    PhoenixUtil.tryUntilOk(CONFIG_ATTEMPTS, () -> cancoder.optimizeBusUtilization());

    double drivePositionFactor = 2.0 * Math.PI * wheelRadiusMeters / hardware.driveRatio();
    SparkBaseConfig driveConfig = createConfig(driveControllerType);
    driveConfig
        .inverted(hardware.driveInverted())
        .idleMode(IdleMode.kBrake)
        .smartCurrentLimit(DriveConstants.DRIVE_CURRENT_LIMIT)
        .voltageCompensation(NOMINAL_VOLTAGE);
    driveConfig
        .encoder
        .positionConversionFactor(drivePositionFactor)
        .velocityConversionFactor(drivePositionFactor / 60.0)
        .uvwMeasurementPeriod(10)
        .uvwAverageDepth(2);
    driveConfig
        .closedLoop
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        .pid(DriveConstants.DRIVE_KP / NOMINAL_VOLTAGE, 0.0, toSparkKd(DriveConstants.DRIVE_KD));
    configureSignals(driveConfig);
    tryUntilOk(
        driveSpark,
        CONFIG_ATTEMPTS,
        () ->
            driveSpark.configure(
                driveConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));
    tryUntilOk(driveSpark, CONFIG_ATTEMPTS, () -> driveEncoder.setPosition(0.0));

    double steerPositionFactor = 2.0 * Math.PI / hardware.steerRatio();
    SparkBaseConfig steerConfig = createConfig(steerControllerType);
    steerConfig
        .inverted(hardware.steerInverted())
        .idleMode(IdleMode.kBrake)
        .smartCurrentLimit(DriveConstants.STEER_CURRENT_LIMIT)
        .voltageCompensation(NOMINAL_VOLTAGE);
    steerConfig
        .encoder
        .positionConversionFactor(steerPositionFactor)
        .velocityConversionFactor(steerPositionFactor / 60.0)
        .uvwMeasurementPeriod(10)
        .uvwAverageDepth(2);
    steerConfig
        .closedLoop
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        .positionWrappingEnabled(true)
        .positionWrappingInputRange(-Math.PI, Math.PI)
        .pid(DriveConstants.STEER_KP / NOMINAL_VOLTAGE, 0.0, toSparkKd(DriveConstants.STEER_KD));
    configureSignals(steerConfig);
    tryUntilOk(
        steerSpark,
        CONFIG_ATTEMPTS,
        () ->
            steerSpark.configure(
                steerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));

    boolean encoderOk =
        PhoenixUtil.tryUntilOk(
            CONFIG_ATTEMPTS,
            () -> absolutePositionSignal.waitForUpdate(CANCODER_TIMEOUT_SECONDS).getStatus());
    if (encoderOk) {
      updateAbsolutePosition();
      double seedRad = absolutePosition.getRadians();
      if (tryUntilOk(steerSpark, CONFIG_ATTEMPTS, () -> steerEncoder.setPosition(seedRad))) {
        steerPositionRad = seedRad;
        steerCouplingOriginRad = seedRad;
      }
    }

    timestampQueue = SparkOdometryThread.getInstance().makeTimestampQueue();
    drivePositionQueue =
        SparkOdometryThread.getInstance().registerSignal(driveSpark, driveEncoder::getPosition);
    steerPositionQueue =
        SparkOdometryThread.getInstance().registerSignal(steerSpark, steerEncoder::getPosition);
  }

  @Override
  public void updateInputs(ModuleIOInputs inputs) {
    SparkUtil.sparkStickyFault = false;
    ifOk(driveSpark, driveEncoder::getPosition, value -> driveRawPositionMeters = value);
    ifOk(driveSpark, driveEncoder::getVelocity, value -> driveRawVelocityMetersPerSec = value);
    ifOk(
        driveSpark,
        new DoubleSupplier[] {driveSpark::getAppliedOutput, driveSpark::getBusVoltage},
        values -> inputs.driveAppliedVolts = values[0] * values[1]);
    ifOk(driveSpark, driveSpark::getOutputCurrent, value -> inputs.driveCurrentAmps = value);
    inputs.driveConnected = driveConnectedDebounce.calculate(!SparkUtil.sparkStickyFault);

    SparkUtil.sparkStickyFault = false;
    ifOk(steerSpark, steerEncoder::getPosition, value -> steerPositionRad = value);
    ifOk(steerSpark, steerEncoder::getVelocity, value -> steerVelocityRadPerSec = value);
    ifOk(
        steerSpark,
        new DoubleSupplier[] {steerSpark::getAppliedOutput, steerSpark::getBusVoltage},
        values -> inputs.steerAppliedVolts = values[0] * values[1]);
    ifOk(steerSpark, steerSpark::getOutputCurrent, value -> inputs.steerCurrentAmps = value);
    boolean steerOk = !SparkUtil.sparkStickyFault;
    inputs.steerConnected = steerConnectedDebounce.calculate(steerOk);

    boolean encoderOk = BaseStatusSignal.refreshAll(absolutePositionSignal).isOK();
    if (encoderOk) {
      updateAbsolutePosition();
    }
    inputs.encoderConnected = encoderConnectedDebounce.calculate(encoderOk);
    inputs.absolutePositionRaw = absolutePositionRaw;
    inputs.absolutePosition = absolutePosition;

    updateSteerResync(encoderOk && steerOk);

    inputs.drivePositionMeters = correctDrivePosition(driveRawPositionMeters, steerPositionRad);
    inputs.driveVelocityMetersPerSec =
        driveRawVelocityMetersPerSec - couplingMetersPerSteerRadian * steerVelocityRadPerSec;
    inputs.steerPosition = new Rotation2d(steerPositionRad);
    inputs.steerVelocityRadPerSec = steerVelocityRadPerSec;
    inputs.steerResyncCount = steerResyncCount;

    Drive.ODOMETRY_LOCK.lock();
    try {
      double[] timestamps = timestampQueue.stream().mapToDouble(Double::doubleValue).toArray();
      double[] drivePositions =
          drivePositionQueue.stream().mapToDouble(Double::doubleValue).toArray();
      double[] steerPositions =
          steerPositionQueue.stream().mapToDouble(Double::doubleValue).toArray();
      timestampQueue.clear();
      drivePositionQueue.clear();
      steerPositionQueue.clear();

      int sampleCount =
          Math.min(timestamps.length, Math.min(drivePositions.length, steerPositions.length));
      inputs.odometryTimestamps = new double[sampleCount];
      inputs.odometryDrivePositionsMeters = new double[sampleCount];
      inputs.odometrySteerPositions = new Rotation2d[sampleCount];
      for (int i = 0; i < sampleCount; i++) {
        inputs.odometryTimestamps[i] = timestamps[i];
        inputs.odometryDrivePositionsMeters[i] =
            correctDrivePosition(drivePositions[i], steerPositions[i]);
        inputs.odometrySteerPositions[i] = new Rotation2d(steerPositions[i]);
      }
    } finally {
      Drive.ODOMETRY_LOCK.unlock();
    }
  }

  @Override
  public void setDriveOpenLoop(double volts) {
    driveSpark.setVoltage(volts);
  }

  @Override
  public void setSteerOpenLoop(double volts) {
    steerSpark.setVoltage(volts);
  }

  @Override
  public void setDriveVelocity(double velocityMetersPerSec, double feedforwardVolts) {
    double rawSetpoint =
        velocityMetersPerSec + couplingMetersPerSteerRadian * steerVelocityRadPerSec;
    driveController.setSetpoint(
        rawSetpoint,
        ControlType.kVelocity,
        ClosedLoopSlot.kSlot0,
        feedforwardVolts,
        ArbFFUnits.kVoltage);
  }

  @Override
  public void setSteerPosition(Rotation2d position) {
    steerController.setSetpoint(
        MathUtil.inputModulus(position.getRadians(), -Math.PI, Math.PI), ControlType.kPosition);
  }

  @Override
  public void setDrivePID(double kP, double kD) {
    SparkBaseConfig config = createConfig(driveControllerType);
    config.closedLoop.pid(kP / NOMINAL_VOLTAGE, 0.0, toSparkKd(kD));
    applyLiveConfig(driveSpark, config);
  }

  @Override
  public void setSteerPID(double kP, double kD) {
    SparkBaseConfig config = createConfig(steerControllerType);
    config.closedLoop.pid(kP / NOMINAL_VOLTAGE, 0.0, toSparkKd(kD));
    applyLiveConfig(steerSpark, config);
  }

  @Override
  public void setDriveBrakeMode(boolean enabled) {
    SparkBaseConfig config = createConfig(driveControllerType);
    config.idleMode(enabled ? IdleMode.kBrake : IdleMode.kCoast);
    applyLiveConfig(driveSpark, config);
  }

  private double correctDrivePosition(double rawPositionMeters, double steerPositionRadians) {
    return rawPositionMeters
        - couplingMetersPerSteerRadian * (steerPositionRadians - steerCouplingOriginRad);
  }

  private void updateAbsolutePosition() {
    absolutePositionRaw = Rotation2d.fromRotations(absolutePositionSignal.getValueAsDouble());
    absolutePosition = absolutePositionRaw.minus(encoderOffset);
  }

  private void updateSteerResync(boolean sensorsOk) {
    double error = MathUtil.angleModulus(steerPositionRad - absolutePosition.getRadians());
    boolean outOfSync =
        sensorsOk
            && Math.abs(steerVelocityRadPerSec)
                < DriveConstants.STEER_RESYNC_MAX_VELOCITY_RAD_PER_SEC
            && Math.abs(error) > DriveConstants.STEER_RESYNC_THRESHOLD_RADIANS;
    if (!outOfSync) {
      steerResyncCycles = 0;
      return;
    }
    steerResyncCycles++;
    if (steerResyncCycles < DriveConstants.STEER_RESYNC_CYCLES) {
      return;
    }
    steerResyncCycles = 0;
    double resyncedPositionRad = steerPositionRad - error;
    if (steerEncoder.setPosition(resyncedPositionRad) == REVLibError.kOk) {
      steerPositionRad = resyncedPositionRad;
      steerResyncCount++;
    }
  }

  private static double toSparkKd(double kDVoltsPerUnitPerSecond) {
    return kDVoltsPerUnitPerSecond / NOMINAL_VOLTAGE * SPARK_KD_MILLISECONDS_PER_SECOND;
  }

  private static void applyLiveConfig(SparkBase spark, SparkBaseConfig config) {
    spark.configureAsync(
        config, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
  }

  private static void configureSignals(SparkBaseConfig config) {
    config
        .signals
        .primaryEncoderPositionAlwaysOn(true)
        .primaryEncoderPositionPeriodMs((int) (1000.0 / DriveConstants.ODOMETRY_FREQUENCY_HZ))
        .primaryEncoderVelocityAlwaysOn(true)
        .primaryEncoderVelocityPeriodMs(STATUS_PERIOD_MS)
        .appliedOutputPeriodMs(STATUS_PERIOD_MS)
        .busVoltagePeriodMs(STATUS_PERIOD_MS)
        .outputCurrentPeriodMs(STATUS_PERIOD_MS);
  }

  private static SparkBase createSpark(int id, DriveConstants.MotorControllerType type) {
    return switch (type) {
      case SPARK_MAX -> new SparkMax(id, MotorType.kBrushless);
      case SPARK_FLEX -> new SparkFlex(id, MotorType.kBrushless);
    };
  }

  private static SparkBaseConfig createConfig(DriveConstants.MotorControllerType type) {
    return switch (type) {
      case SPARK_MAX -> new SparkMaxConfig();
      case SPARK_FLEX -> new SparkFlexConfig();
    };
  }
}
