package frc.robot.subsystems;

import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants;
import java.util.Timer;
import java.util.TimerTask;

public class ControllerSubsystem extends SubsystemBase {

  public enum ControllerName {
    DRIVE,
    MANIP
  }

  public enum Button {
    A(1),
    B(2),
    X(3),
    Y(4),
    LB(5),
    RB(6),
    BACK(7),
    START(8),
    LS(9),
    RS(10);

    public final int id;

    Button(int id) {
      this.id = id;
    }
  }

  public enum Axis {
    LX(0),
    LY(1),
    LT(2),
    RT(3),
    RX(4),
    RY(5);

    public final int id;

    Axis(int id) {
      this.id = id;
    }
  }

  private static ControllerSubsystem instance;

  private final CommandXboxController driveCommandController;
  private final CommandXboxController manipCommandController;
  private final XboxController driveHid;
  private final XboxController manipHid;

  private ControllerSubsystem() {
    driveCommandController = new CommandXboxController(Constants.Operator.DRIVE_CONTROLLER_PORT);
    manipCommandController = new CommandXboxController(Constants.Operator.MANIP_CONTROLLER_PORT);
    driveHid = driveCommandController.getHID();
    manipHid = manipCommandController.getHID();
  }

  public static ControllerSubsystem getInstance() {
    if (instance == null) {
      instance = new ControllerSubsystem();
    }
    return instance;
  }

  public CommandXboxController getCommandController(ControllerName name) {
    return name == ControllerName.DRIVE ? driveCommandController : manipCommandController;
  }

  private XboxController getHid(ControllerName name) {
    return name == ControllerName.DRIVE ? driveHid : manipHid;
  }

  public boolean get(ControllerName name, Button button) {
    return getHid(name).getRawButton(button.id);
  }

  public double get(ControllerName name, Axis axis) {
    return getHid(name).getRawAxis(axis.id);
  }

  public Trigger getTrigger(ControllerName name, Button button) {
    return getCommandController(name).button(button.id);
  }

  public int getDPAD(ControllerName name) {
    return getHid(name).getPOV();
  }

  public void setRumble(ControllerName name, double intensity) {
    getHid(name).setRumble(RumbleType.kBothRumble, intensity);
  }

  public void vibrate(ControllerName name) {
    vibrate(name, Constants.Operator.RUMBLE_DURATION_MILLIS, Constants.Operator.RUMBLE_INTENSITY);
  }

  public void vibrate(ControllerName name, long millis, double intensity) {
    setRumble(name, intensity);
    Timer timer = new Timer(true);
    timer.schedule(
        new TimerTask() {
          @Override
          public void run() {
            setRumble(name, 0.0);
            timer.cancel();
          }
        },
        millis);
  }
}
