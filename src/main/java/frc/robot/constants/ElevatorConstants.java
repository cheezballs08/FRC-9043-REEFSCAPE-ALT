package frc.robot.constants;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

public class ElevatorConstants {
  
  public static final SparkBaseConfig motorDefaultConfig = new SparkMaxConfig()
  .smartCurrentLimit(0)
  .idleMode(IdleMode.kBrake);
  
  public static final int motor1ID = 0;
  public static final MotorType motor1type = MotorType.kBrushless;
  public static final SparkBaseConfig motor1Config = new SparkMaxConfig()
  .apply(motorDefaultConfig)
  .inverted(false);
    
  public static final int motor2ID = 0;
  public static final MotorType motor2type = MotorType.kBrushless;
  public static final SparkBaseConfig motor2Config = new SparkMaxConfig()
  .apply(motorDefaultConfig)
  .inverted(false);
        
  public static final int encoderID = 0;

  public static final double encoderSpeedConversionFactor = 0.0;
  public static final double encoderAccelerationConversionFactor = 0.0;

  public static final double P = 0.0;
  public static final double I = 0.0;
  public static final double IZ = 0.0;
  public static final double D = 0.0;

  public static final double elevatorHeight = 0.0;

  public static final double L1Height = 0.0;
  public static final double L2Height = 0.0;
  public static final double L3Height = 0.0;
  public static final double L4Height = 0.0;
}
