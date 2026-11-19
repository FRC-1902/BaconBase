package frc.robot.util;

import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.wpilibj.Timer;
import frc.robot.subsystems.drive.DriveConstants;
import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.drivesims.COTS;
import org.ironmaple.simulation.drivesims.configs.DriveTrainSimulationConfig;
import org.ironmaple.simulation.drivesims.configs.SwerveModuleSimulationConfig;

public final class MapleSimUtil {

  private static final double STEER_FRICTION_VOLTS = 0.2;

  private MapleSimUtil() {}

  public static DriveTrainSimulationConfig driveTrainConfig() {
    var hardware = DriveConstants.HARDWARE;
    return DriveTrainSimulationConfig.Default()
        .withRobotMass(DriveConstants.ROBOT_MASS)
        .withBumperSize(DriveConstants.BUMPER_LENGTH, DriveConstants.BUMPER_WIDTH)
        .withCustomModuleTranslations(DriveConstants.MODULE_TRANSLATIONS)
        .withGyro(COTS.ofPigeon2())
        .withSwerveModule(
            new SwerveModuleSimulationConfig(
                hardware.driveMotor(),
                hardware.steerMotor(),
                hardware.driveRatio(),
                hardware.steerRatio(),
                Volts.of(DriveConstants.DRIVE_KS),
                Volts.of(STEER_FRICTION_VOLTS),
                DriveConstants.WHEEL_RADIUS,
                hardware.steerInertia(),
                DriveConstants.WHEEL_COF));
  }

  public static double[] getSimulationOdometryTimeStamps() {
    int subTicks = SimulatedArena.getSimulationSubTicksIn1Period();
    double dt = SimulatedArena.getSimulationDt().in(Seconds);
    double now = Timer.getFPGATimestamp();
    double[] timestamps = new double[subTicks];
    for (int i = 0; i < subTicks; i++) {
      timestamps[i] = now - (subTicks - 1 - i) * dt;
    }
    return timestamps;
  }
}
