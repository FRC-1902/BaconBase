package frc.robot;

import static edu.wpi.first.units.Units.Seconds;

import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import org.littletonrobotics.junction.LogFileUtil;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGReader;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;
import org.littletonrobotics.urcl.URCL;

public class Robot extends LoggedRobot {

  private final Timer wheelLockTimer = new Timer();
  private final Timer batteryCheckTimer = new Timer();

  private final Alert lowBatteryAlert = new Alert("Low Battery", AlertType.kWarning);
  private final Alert criticalBatteryAlert = new Alert("Critical Battery", AlertType.kError);

  private RobotContainer robotContainer;
  private Command autonomousCommand;

  @SuppressWarnings("resource")
  @Override
  public void robotInit() {
    Logger.recordMetadata("ProjectName", Constants.PROJECT_NAME);

    switch (Constants.CURRENT_MODE) {
      case REAL -> {
        Logger.addDataReceiver(new WPILOGWriter());
        Logger.addDataReceiver(new NT4Publisher());
        Logger.registerURCL(URCL.startExternal());
        new PowerDistribution(Constants.Power.PDH_ID, ModuleType.kRev);
      }
      case SIM -> Logger.addDataReceiver(new NT4Publisher());
      case REPLAY -> {
        setUseTiming(false);
        String logPath = LogFileUtil.findReplayLog();
        Logger.setReplaySource(new WPILOGReader(logPath));
        Logger.addDataReceiver(new WPILOGWriter(LogFileUtil.addPathSuffix(logPath, "_replay")));
      }
    }

    Logger.start();
    DataLogManager.start();

    robotContainer = new RobotContainer();

    SmartDashboard.putData(CommandScheduler.getInstance());

    if (isSimulation()) {
      DriverStation.silenceJoystickConnectionWarning(true);
    }

    batteryCheckTimer.start();
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
    robotContainer.updateVision();
    robotContainer.logDriverInputs();
    checkBatteryVoltage();
  }

  @Override
  public void disabledInit() {
    robotContainer.setMotorBrake(true);
    wheelLockTimer.reset();
    wheelLockTimer.start();
  }

  @Override
  public void disabledPeriodic() {
    if (wheelLockTimer.hasElapsed(Constants.Drive.WHEEL_LOCK_TIME.in(Seconds))) {
      robotContainer.setMotorBrake(false);
      wheelLockTimer.stop();
      wheelLockTimer.reset();
    }
  }

  @Override
  public void autonomousInit() {
    robotContainer.setMotorBrake(true);
    autonomousCommand = robotContainer.getAutonomousCommand();
    if (autonomousCommand != null) {
      CommandScheduler.getInstance().schedule(autonomousCommand);
    }
  }

  @Override
  public void teleopInit() {
    if (autonomousCommand != null) {
      autonomousCommand.cancel();
    } else {
      CommandScheduler.getInstance().cancelAll();
    }
    robotContainer.setMotorBrake(true);
  }

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
  }

  private void checkBatteryVoltage() {
    if (!batteryCheckTimer.hasElapsed(Constants.Power.BATTERY_CHECK_INTERVAL)) {
      return;
    }
    batteryCheckTimer.reset();

    double voltage = RobotController.getBatteryVoltage();
    boolean critical = voltage < Constants.Power.BATTERY_CRITICAL_VOLTAGE;
    boolean low = !critical && voltage < Constants.Power.BATTERY_WARNING_VOLTAGE;

    criticalBatteryAlert.set(critical);
    lowBatteryAlert.set(low);
  }
}
