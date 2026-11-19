package frc.robot.util;

import frc.robot.Constants;
import java.util.Arrays;
import java.util.HashMap;
import java.util.Map;
import java.util.function.Consumer;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

public class LoggedTunableNumber implements DoubleSupplier {

  private static final String TABLE_KEY = "/Tuning";

  private final String key;

  private final double defaultValue;

  private final Map<Integer, Double> lastHasChangedValues = new HashMap<>();

  private LoggedNetworkNumber dashboardNumber;

  public LoggedTunableNumber(String dashboardKey, double defaultValue) {
    this.key = TABLE_KEY + "/" + dashboardKey;
    this.defaultValue = defaultValue;
    if (Constants.TUNING_MODE) {
      dashboardNumber = new LoggedNetworkNumber(key, defaultValue);
    }
  }

  public double get() {
    return Constants.TUNING_MODE ? dashboardNumber.get() : defaultValue;
  }

  public boolean hasChanged(int id) {
    double currentValue = get();
    Double lastValue = lastHasChangedValues.get(id);
    if (lastValue == null || currentValue != lastValue) {
      lastHasChangedValues.put(id, currentValue);
      return true;
    }
    return false;
  }

  public static void ifChanged(
      int id, Consumer<double[]> action, LoggedTunableNumber... tunableNumbers) {
    boolean changed = false;
    for (LoggedTunableNumber tunableNumber : tunableNumbers) {
      changed |= tunableNumber.hasChanged(id);
    }
    if (changed) {
      action.accept(Arrays.stream(tunableNumbers).mapToDouble(LoggedTunableNumber::get).toArray());
    }
  }

  @Override
  public double getAsDouble() {
    return get();
  }
}
