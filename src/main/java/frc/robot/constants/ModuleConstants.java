package frc.robot.constants;

import com.pathplanner.lib.config.ModuleConfig;
import com.pathplanner.lib.config.PIDConstants;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.system.plant.DCMotor;

public class ModuleConstants {
  public static final double wheelDiameter = 0;

  public static final double wheelCoefficientOfFriction = 0;

  public static final MotorType driveMotorType = MotorType.kBrushless;
  public static final MotorType angleMotorType = MotorType.kBrushless;

  public static final IdleMode idleMode = IdleMode.kBrake;

  public static final int driveCurrentLimit = 0;
  public static final int angleCurrentLimit = 0;

  public static final SparkBaseConfig driveMotorDefaultConfig = new SparkMaxConfig()
  .smartCurrentLimit(driveCurrentLimit)
  .idleMode(idleMode);
      
  public static final SparkBaseConfig angleMotorDefaultConfig = new SparkMaxConfig()
  .smartCurrentLimit(angleCurrentLimit)
  .idleMode(idleMode);

  // Meters
  public static final double driveMaxSpeed = 0;
  public static final double driveMaxAcceleration = 0;

  // Radians
  public static final double angleMaxSpeed = 0;
  public static final double angleMaxAcceleration = 0;

  public static final double driveMotorGearRatio = 0;
  // Rotation to meters.
  public static final double driveEncoderSpeedConversionFactor = 0;
  // Rotation per minute to meters per second
  public static final double driveEncoderAcclerationConversionFactor = 0;

  public static final double angleMotorGearRatio = 0;
  // Rotations to radians
  public static final double angleEncoderSpeedConversionFactor = 0;
  // rotations per minute to radians per second
  public static final double angleEncoderAccelerationConversionFactor = 0;

  public static final double PDrive = 0;
  public static final double IDrive = 0;
  public static final double DDrive = 0;
  public static final double IZDrive = 0;

  public static final PIDConstants drivePID = new PIDConstants(PDrive, IDrive, DDrive, IZDrive);

  public static final double PAngle = 0;
  public static final double IAngle = 0;
  public static final double DAngle = 0;
  public static final double IZAngle = 0;

  public static final PIDConstants anglePID = new PIDConstants(PAngle, IAngle, DAngle, IZAngle);

  // TODO: Modül başına motor neden 1? (Büyük ihtimal sadece sürüş motorunu hesaba kattığım.)
  // TODO: Gear reduction ile Gear ratio aynı şey mi bak.
  public static final ModuleConfig moduleConfig = new ModuleConfig(
      wheelDiameter / 2,
      driveMaxSpeed, 
      wheelCoefficientOfFriction, 
      DCMotor.getNEO(1).withReduction(driveMotorGearRatio), 
      driveCurrentLimit, 
      1
  );
}
