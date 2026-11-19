package frc.robot.subsystems.drive;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.hardware.Pigeon2;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import frc.robot.util.PhoenixUtil;
import java.util.Queue;

public class GyroIOPigeon2 implements GyroIO {

  private static final int CONFIG_ATTEMPTS = 5;

  private static final double CONFIG_TIMEOUT_SECONDS = 0.25;

  private static final double YAW_VELOCITY_UPDATE_FREQUENCY_HZ = 50.0;

  private final Pigeon2 pigeon = new Pigeon2(DriveConstants.PIGEON_ID, DriveConstants.CAN_BUS);

  private final StatusSignal<Angle> yaw = pigeon.getYaw();

  private final StatusSignal<AngularVelocity> yawVelocity = pigeon.getAngularVelocityZWorld();

  private final Queue<Double> yawTimestampQueue;

  private final Queue<Double> yawPositionQueue;

  public GyroIOPigeon2() {
    PhoenixUtil.tryUntilOk(
        CONFIG_ATTEMPTS,
        () -> pigeon.getConfigurator().apply(new Pigeon2Configuration(), CONFIG_TIMEOUT_SECONDS));
    PhoenixUtil.tryUntilOk(
        CONFIG_ATTEMPTS, () -> pigeon.getConfigurator().setYaw(0.0, CONFIG_TIMEOUT_SECONDS));
    PhoenixUtil.tryUntilOk(
        CONFIG_ATTEMPTS, () -> yaw.setUpdateFrequency(DriveConstants.ODOMETRY_FREQUENCY_HZ));
    PhoenixUtil.tryUntilOk(
        CONFIG_ATTEMPTS, () -> yawVelocity.setUpdateFrequency(YAW_VELOCITY_UPDATE_FREQUENCY_HZ));
    PhoenixUtil.tryUntilOk(CONFIG_ATTEMPTS, () -> pigeon.optimizeBusUtilization());

    yawTimestampQueue = SparkOdometryThread.getInstance().makeTimestampQueue();
    StatusSignal<Angle> yawClone = yaw.clone();
    yawPositionQueue =
        SparkOdometryThread.getInstance()
            .registerSignal(() -> yawClone.refresh().getValueAsDouble());
  }

  @Override
  public void updateInputs(GyroIOInputs inputs) {
    inputs.connected = BaseStatusSignal.refreshAll(yaw, yawVelocity).isOK();
    inputs.yawPosition = Rotation2d.fromDegrees(yaw.getValueAsDouble());
    inputs.yawVelocityRadPerSec = Units.degreesToRadians(yawVelocity.getValueAsDouble());

    Drive.ODOMETRY_LOCK.lock();
    try {
      inputs.odometryYawTimestamps =
          yawTimestampQueue.stream().mapToDouble(Double::doubleValue).toArray();
      inputs.odometryYawPositions =
          yawPositionQueue.stream().map(Rotation2d::fromDegrees).toArray(Rotation2d[]::new);
      yawTimestampQueue.clear();
      yawPositionQueue.clear();
    } finally {
      Drive.ODOMETRY_LOCK.unlock();
    }
  }
}
