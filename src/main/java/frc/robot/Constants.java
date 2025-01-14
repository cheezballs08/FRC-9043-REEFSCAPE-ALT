package frc.robot;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.studica.frc.AHRS.NavXComType;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;

public class Constants {

  public static class ModuleConsants {
    public static final double wheelDiameter = 0;
    
    public static final double driveMaxSpeed = 0;
    public static final double driveMaxAcceleration = 0;
    
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
      
    public static final double PAngle = 0;
    public static final double IAngle = 0;
    public static final double DAngle = 0;
    public static final double IZAngle = 0;
  }

  public class DrivetrainConstants {
    public static final int frontLeftDriveMotorID = 0;
    public static final MotorType frontLeftDriveMotorType = MotorType.kBrushless;
    public static final boolean frontLeftInvertDriveMotor = false;
    public static final boolean frontLeftInvertDriveEncoder = false;

    public static final int frontLeftAngleMotorID = 0;
    public static final MotorType frontLeftAngleMotorType = MotorType.kBrushless;
    public static final boolean frontLeftInvertAngleMotor = false;
    public static final boolean frontLeftInvertAngleEncoder = false;

    public static final int frontLeftAbsoluteEncoderID = 0;
    public static final double frontLeftAbsoluteEncoderOffset = 0.0;
    public static final boolean frontLeftInvertAbsoluteEncoder = false;

    
    public static final int frontRightDriveMotorID = 0;
    public static final MotorType frontRightDriveMotorType = MotorType.kBrushless;
    public static final boolean frontRightInvertDriveMotor = false;
    public static final boolean frontRightInvertDriveEncoder = false;

    public static final int frontRightAngleMotorID = 0;
    public static final MotorType frontRightAngleMotorType = MotorType.kBrushless;
    public static final boolean frontRightInvertAngleMotor = false;
    public static final boolean frontRightInvertAngleEncoder = false;

    public static final int frontRightAbsoluteEncoderID = 0;
    public static final double frontRightAbsoluteEncoderOffset = 0.0;
    public static final boolean frontRightInvertAbsoluteEncoder = false;  


    public static final int backLeftDriveMotorID = 0;
    public static final MotorType backLeftDriveMotorType = MotorType.kBrushless;
    public static final boolean backLeftInvertDriveMotor = false;
    public static final boolean backLeftInvertDriveEncoder = false;

    public static final int backLeftAngleMotorID = 0;
    public static final MotorType backLeftAngleMotorType = MotorType.kBrushless;
    public static final boolean backLeftInvertAngleMotor = false;
    public static final boolean backLeftInvertAngleEncoder = false;

    public static final int backLeftAbsoluteEncoderID = 0;
    public static final double backLeftAbsoluteEncoderOffset = 0.0;
    public static final boolean backLeftInvertAbsoluteEncoder = false;


    public static final int backRightDriveMotorID = 0;
    public static final MotorType backRightDriveMotorType = MotorType.kBrushless;
    public static final boolean backRightInvertDriveMotor = false;
    public static final boolean backRightInvertDriveEncoder = false;

    public static final int backRightAngleMotorID = 0;
    public static final MotorType backRightAngleMotorType = MotorType.kBrushless;
    public static final boolean backRightInvertAngleMotor = false;
    public static final boolean backRightInvertAngleEncoder = false;

    public static final int backRightAbsoluteEncoderID = 0;
    public static final double backRightAbsoluteEncoderOffset = 0.0;
    public static final boolean backRightInvertAbsoluteEncoder = false;

    public static final NavXComType GyroscopeCommunicationType = NavXComType.kI2C;

    // Centimeters
    public static final double moduleLateralDistance = 0.0;
    public static final double moduleLongitudinalDistance = 0.0;

    // This is probably very wrong so :P
    // TODO: Check if this is actually true
    public static final SwerveDriveKinematics kinematics = new SwerveDriveKinematics(
      new Translation2d(moduleLateralDistance / 2, moduleLongitudinalDistance / 2), // Front Left
      new Translation2d(-moduleLateralDistance / 2, moduleLongitudinalDistance / 2), // Front Right
      new Translation2d(moduleLateralDistance / 2, -moduleLongitudinalDistance / 2), // Back Left
      new Translation2d(-moduleLateralDistance / 2, -moduleLongitudinalDistance / 2) // Back Right
    );

  }
  
  public static class RobotConsants {
    public static final Pose2d initialPose = new Pose2d();
    
  }

  public static class ControllerConsants {
    public static final double deadband = 0.0;

    public static final double driveSpeedReductionFactor = 0.0;
    public static final double angleSpeedReductionFactor = 0.0;

    public static final double maxAllowedDriveAcceleration = 0.0;
    public static final double maxAllowedAngleAcceleration = 0.0;
  }


  public static class MathConsants {
    public static final double epsilon = 0.001;
  }

  public static class TODOConstants {
    public static final double TODOConstantDouble = 0.0;
    public static final int TODOConstantInt = 0;
    public static final boolean TODOConstantBoolean = false;
  }
}