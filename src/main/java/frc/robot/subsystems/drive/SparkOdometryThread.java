package frc.robot.subsystems.drive;

import com.revrobotics.REVLibError;
import com.revrobotics.spark.SparkBase;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotController;
import java.util.ArrayList;
import java.util.List;
import java.util.Queue;
import java.util.concurrent.ArrayBlockingQueue;
import java.util.function.DoubleSupplier;

public final class SparkOdometryThread {

  private static final int QUEUE_CAPACITY = 20;

  private static SparkOdometryThread instance = null;

  private final List<SparkBase> sparks = new ArrayList<>();

  private final List<DoubleSupplier> sparkSignals = new ArrayList<>();

  private final List<Queue<Double>> sparkQueues = new ArrayList<>();

  private final List<DoubleSupplier> genericSignals = new ArrayList<>();

  private final List<Queue<Double>> genericQueues = new ArrayList<>();

  private final List<Queue<Double>> timestampQueues = new ArrayList<>();

  private final Notifier notifier = new Notifier(this::run);

  private SparkOdometryThread() {
    notifier.setName("OdometryThread");
  }

  public static synchronized SparkOdometryThread getInstance() {
    if (instance == null) {
      instance = new SparkOdometryThread();
    }
    return instance;
  }

  public void start() {
    boolean hasSignals;
    Drive.ODOMETRY_LOCK.lock();
    try {
      hasSignals = !timestampQueues.isEmpty();
    } finally {
      Drive.ODOMETRY_LOCK.unlock();
    }
    if (hasSignals) {
      notifier.startPeriodic(1.0 / DriveConstants.ODOMETRY_FREQUENCY_HZ);
    }
  }

  public Queue<Double> registerSignal(SparkBase spark, DoubleSupplier signal) {
    Queue<Double> queue = new ArrayBlockingQueue<>(QUEUE_CAPACITY);
    Drive.ODOMETRY_LOCK.lock();
    try {
      sparks.add(spark);
      sparkSignals.add(signal);
      sparkQueues.add(queue);
    } finally {
      Drive.ODOMETRY_LOCK.unlock();
    }
    return queue;
  }

  public Queue<Double> registerSignal(DoubleSupplier signal) {
    Queue<Double> queue = new ArrayBlockingQueue<>(QUEUE_CAPACITY);
    Drive.ODOMETRY_LOCK.lock();
    try {
      genericSignals.add(signal);
      genericQueues.add(queue);
    } finally {
      Drive.ODOMETRY_LOCK.unlock();
    }
    return queue;
  }

  public Queue<Double> makeTimestampQueue() {
    Queue<Double> queue = new ArrayBlockingQueue<>(QUEUE_CAPACITY);
    Drive.ODOMETRY_LOCK.lock();
    try {
      timestampQueues.add(queue);
    } finally {
      Drive.ODOMETRY_LOCK.unlock();
    }
    return queue;
  }

  private void run() {
    Drive.ODOMETRY_LOCK.lock();
    try {
      double timestamp = RobotController.getFPGATime() / 1e6;
      double[] sparkValues = new double[sparkSignals.size()];
      boolean isValid = true;
      for (int i = 0; i < sparkSignals.size(); i++) {
        sparkValues[i] = sparkSignals.get(i).getAsDouble();
        if (sparks.get(i).getLastError() != REVLibError.kOk) {
          isValid = false;
        }
      }
      if (isValid) {
        for (int i = 0; i < sparkSignals.size(); i++) {
          sparkQueues.get(i).offer(sparkValues[i]);
        }
        for (int i = 0; i < genericSignals.size(); i++) {
          genericQueues.get(i).offer(genericSignals.get(i).getAsDouble());
        }
        for (Queue<Double> timestampQueue : timestampQueues) {
          timestampQueue.offer(timestamp);
        }
      }
    } finally {
      Drive.ODOMETRY_LOCK.unlock();
    }
  }
}
