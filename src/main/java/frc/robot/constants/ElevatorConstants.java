package frc.robot.constants;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;

import com.revrobotics.spark.config.SparkMaxConfig;

public class ElevatorConstants {


  public static final double elevatorHeight = 1.5;
  public static final double L1Height = 0.25;
  public static final double L2Height = 0.50;
  public static final double L3Height = 0.75;
  public static final double L4Height = 0.1;
  public static final double feedHeight = 1.25;
  public static final double restHeight = 0;

  public static final double startingHeight = restHeight;
  
  public static final double elevatorMass = 22;
  
  public static final double drumRadius = 0.03343;

  public static final double gearing = 11.28;
  
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

  public static final double encoderPositionConversionFactor = 0.0;
  public static final double encoderVelocityConversionFactor = 0.0;

  public static final double P = 10;
  public static final double I = 0.0;
  public static final double IZ = 0.0;
  public static final double D = 0.0;

  public static final Constraints constraints = new Constraints(100, 100);

  public static final double S = 0;
  public static final double G = 0;
  public static final double V = 0;
  public static final double A = 0;

}
