package frc.robot.constants;

import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.studica.frc.AHRS.NavXComType;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;

public class DrivetrainConstants {
  
  // * Front Left

  public static final int frontLeftDriveMotorID = 0;
  public static final SparkBaseConfig frontLeftDriveMotorConfig = new SparkMaxConfig()
  .inverted(false)
  .apply(ModuleConstants.driveMotorDefaultConfig);
  public static final boolean frontLeftInvertDriveEncoder = false;

  public static final int frontLeftAngleMotorID = 0;
  public static final SparkBaseConfig frontLeftAngleMotorConfig = new SparkMaxConfig()
  .inverted(false)
  .apply(ModuleConstants.angleMotorDefaultConfig);
  public static final boolean frontLeftInvertAngleEncoder = false;

  public static final int frontLeftAbsoluteEncoderID = 0;
  public static final double frontLeftAbsoluteEncoderOffset = 0.0;
  public static final boolean frontLeftInvertAbsoluteEncoder = false;

  // * Front Left

  // * Front Right

  public static final int frontRightDriveMotorID = 0;
  public static final SparkBaseConfig frontRightDriveMotorConfig = new SparkMaxConfig()
  .inverted(false)
  .apply(ModuleConstants.driveMotorDefaultConfig);
  public static final boolean frontRightInvertDriveEncoder = false;

  public static final int frontRightAngleMotorID = 0;
  public static final SparkBaseConfig frontRightAngleMotorConfig = new SparkMaxConfig()
  .inverted(false)
  .apply(ModuleConstants.angleMotorDefaultConfig);
  public static final boolean frontRightInvertAngleEncoder = false;

  public static final int frontRightAbsoluteEncoderID = 0;
  public static final double frontRightAbsoluteEncoderOffset = 0.0;
  public static final boolean frontRightInvertAbsoluteEncoder = false;  

  // * Front Right

  // * Back Left

  public static final int backLeftDriveMotorID = 0;
  public static final SparkBaseConfig backLeftDriveMotorConfig = new SparkMaxConfig()
  .inverted(false)
  .apply(ModuleConstants.driveMotorDefaultConfig);
  public static final boolean backLeftInvertDriveEncoder = false;

  public static final int backLeftAngleMotorID = 0;
  public static final SparkBaseConfig backLeftAngleMotorConfig = new SparkMaxConfig()
  .inverted(false)
  .apply(ModuleConstants.angleMotorDefaultConfig);
  public static final boolean backLeftInvertAngleEncoder = false;

  public static final int backLeftAbsoluteEncoderID = 0;
  public static final double backLeftAbsoluteEncoderOffset = 0.0;
  public static final boolean backLeftInvertAbsoluteEncoder = false;

  // * Back Left

  // * Back Right

  public static final int backRightDriveMotorID = 0;
  public static final SparkBaseConfig backRightDriveMotorConfig = new SparkMaxConfig()
  .inverted(false)
  .apply(ModuleConstants.driveMotorDefaultConfig);
  public static final boolean backRightInvertDriveEncoder = false;

  public static final int backRightAngleMotorID = 0;
  public static final SparkBaseConfig backRightAngleMotorConfig = new SparkMaxConfig()
  .inverted(false)
  .apply(ModuleConstants.angleMotorDefaultConfig);
  public static final boolean backRightInvertAngleEncoder = false;

  public static final int backRightAbsoluteEncoderID = 0;
  public static final double backRightAbsoluteEncoderOffset = 0.0;
  public static final boolean backRightInvertAbsoluteEncoder = false;

  // * Back Right

  // * Gyroscope

  public static final NavXComType GyroscopeCommunicationType = NavXComType.kI2C;
  public static final boolean invertGyroscope = false;

  // TODO: Bu değerler cm mi metre mi olsun?
  public static final double moduleLateralDistance = 0.0;
  public static final double moduleLongitudinalDistance = 0.0;

  public static final SwerveDriveKinematics kinematics = new SwerveDriveKinematics(
      new Translation2d(moduleLateralDistance / 2, moduleLongitudinalDistance / 2), // Front Left
      new Translation2d(-moduleLateralDistance / 2, moduleLongitudinalDistance / 2), // Front Right
      new Translation2d(moduleLateralDistance / 2, -moduleLongitudinalDistance / 2), // Back Left
      new Translation2d(-moduleLateralDistance / 2, -moduleLongitudinalDistance / 2) // Back Right
  );
 
}
