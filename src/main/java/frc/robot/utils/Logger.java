package frc.robot.utils;

import dev.doglog.DogLog;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Logger extends DogLog {
  
  public static void log(String key, Sendable value) {
    SmartDashboard.putData(key, value);
  }

}
