package frc.robot.constants;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;

public class CoralIntakeConstants {

  public static final double gearing = 20;
  
  public static final double intakeLength = 0.2;
  
  public static final double intakeWidth = 0.02;

  public static final double intakeMass = 5;
  
  public static final double momentOfInertia = SingleJointedArmSim.estimateMOI(intakeLength, intakeMass);

  public static final double startingAngle = 0;

  public static final double maximumAngle = Units.degreesToRadians(180);

  public static final double minimumAngle = Units.degreesToRadians(0);

  public static final MotorType motorType = MotorType.kBrushless; 

  public static final SparkBaseConfig motorDefaultConfig = new SparkMaxConfig()
  .smartCurrentLimit(0)
  .idleMode(IdleMode.kBrake);
  
  public static final int intakeMotor1ID = 0;
  public static final SparkBaseConfig intakeMotor1Config = new SparkMaxConfig()
  .apply(motorDefaultConfig)
  .inverted(false);

  public static final int intakeMotor2ID = 0;
  public static final SparkBaseConfig intakeMotor2Config = new SparkMaxConfig()
  .apply(motorDefaultConfig)
  .inverted(false);
  
  public static final int angleMotorID = 0;
  public static final SparkBaseConfig angleMotorConfig = new SparkMaxConfig()
  .apply(motorDefaultConfig)
  .inverted(false);

  public static final double intakeSpeed = 0.0;
  
  public static final double outtakeSpeed = 0.0;
  
  public static final double L1Angle = 0;
  public static final double L2Angle = 60;
  public static final double L3Angle = 120;
  public static final double L4Angle = 180;
  
  public static final int sensorID = 0; 
  
  public static final int encoderID = 0;
  
  public static final double positionConversionConstant = 0;
  public static final double velocityConversionConstant = 0;
  
  public static final double P = 1000;
  public static final double I = 10;
  public static final double D = 0;
  public static final double IZ = 5;
  
  public static final Constraints constraints = new Constraints(1000, 1000);
}
