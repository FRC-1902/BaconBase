package frc.robot.util;

import com.ctre.phoenix6.StatusCode;
import java.util.function.Supplier;

public final class PhoenixUtil {

  private PhoenixUtil() {}

  public static boolean tryUntilOk(int maxAttempts, Supplier<StatusCode> command) {
    for (int i = 0; i < maxAttempts; i++) {
      if (command.get().isOK()) {
        return true;
      }
    }
    return false;
  }
}
