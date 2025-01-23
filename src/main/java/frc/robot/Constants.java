package frc.robot;

import org.photonvision.PhotonPoseEstimator.PoseStrategy;

import com.pathplanner.lib.config.ModuleConfig;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.studica.frc.AHRS.NavXComType;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.system.plant.DCMotor;

// TODO: Initialize all of the constants seen here.
public class Constants {

  public static class ModuleConstants {
    public static final double wheelDiameter = 0;

    public static final double wheelCoefficientOfFriction = 0;

    public static final MotorType driveMotorType = MotorType.kBrushless;
    public static final MotorType angleMotorType = MotorType.kBrushless;

    public static final PersistMode persistMode = PersistMode.kPersistParameters;
    public static final ResetMode resetMode = ResetMode.kNoResetSafeParameters;

    public static final int driveCurrentLimit = 0;
    public static final int angleCurrentLimit = 0;

    public static final SparkBaseConfig driveMotorDefaultConfig = new SparkMaxConfig().smartCurrentLimit(driveCurrentLimit);    
    public static final SparkBaseConfig angleMotorDefaultConfig = new SparkMaxConfig().smartCurrentLimit(angleCurrentLimit);
    
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
  
    // TODO: Check why is the motor per module 1? (Probably because it only counts drive Motors)
    // TODO: Check if gear reduction is what I think it is.
    public static final ModuleConfig moduleConfig = new ModuleConfig(
      wheelDiameter / 2,
      driveMaxSpeed, 
      wheelCoefficientOfFriction, 
      DCMotor.getNEO(1).withReduction(driveMotorGearRatio), 
      driveCurrentLimit, 
      1
    );
  }

  public class DrivetrainConstants {
    // * Front Left

    public static final int frontLeftDriveMotorID = 0;
    public static final MotorType frontLeftDriveMotorType = MotorType.kBrushless;
    public static final SparkBaseConfig frontLeftDriveMotorConfig = new SparkMaxConfig()
    .idleMode(null)
    .inverted(false)
    .apply(ModuleConstants.driveMotorDefaultConfig);
    public static final boolean frontLeftInvertDriveEncoder = false;

    public static final int frontLeftAngleMotorID = 0;
    public static final MotorType frontLeftAngleMotorType = MotorType.kBrushless;
    public static final SparkBaseConfig frontLeftAngleMotorConfig = new SparkMaxConfig()
    .idleMode(null)
    .inverted(false)
    .apply(ModuleConstants.angleMotorDefaultConfig);
    public static final boolean frontLeftInvertAngleEncoder = false;

    public static final int frontLeftAbsoluteEncoderID = 0;
    public static final double frontLeftAbsoluteEncoderOffset = 0.0;
    public static final boolean frontLeftInvertAbsoluteEncoder = false;

    // * Front Left

    // * Front Right

    public static final int frontRightDriveMotorID = 0;
    public static final MotorType frontRightDriveMotorType = MotorType.kBrushless;
    public static final SparkBaseConfig frontRightDriveMotorConfig = new SparkMaxConfig()
    .idleMode(null)
    .inverted(false)
    .apply(ModuleConstants.driveMotorDefaultConfig);
    public static final boolean frontRightInvertDriveEncoder = false;

    public static final int frontRightAngleMotorID = 0;
    public static final MotorType frontRightAngleMotorType = MotorType.kBrushless;
    public static final SparkBaseConfig frontRightAngleMotorConfig = new SparkMaxConfig()
    .idleMode(null)
    .inverted(false)
    .apply(ModuleConstants.angleMotorDefaultConfig);
    public static final boolean frontRightInvertAngleEncoder = false;

    public static final int frontRightAbsoluteEncoderID = 0;
    public static final double frontRightAbsoluteEncoderOffset = 0.0;
    public static final boolean frontRightInvertAbsoluteEncoder = false;  

    // * Front Right

    // * Back Left

    public static final int backLeftDriveMotorID = 0;
    public static final MotorType backLeftDriveMotorType = MotorType.kBrushless;
    public static final SparkBaseConfig backLeftDriveMotorConfig = new SparkMaxConfig()
    .idleMode(null)
    .inverted(false)
    .apply(ModuleConstants.driveMotorDefaultConfig);
    public static final boolean backLeftInvertDriveEncoder = false;

    public static final int backLeftAngleMotorID = 0;
    public static final MotorType backLeftAngleMotorType = MotorType.kBrushless;
    public static final SparkBaseConfig backLeftAngleMotorConfig = new SparkMaxConfig()
    .idleMode(null)
    .inverted(false)
    .apply(ModuleConstants.angleMotorDefaultConfig);
    public static final boolean backLeftInvertAngleEncoder = false;

    public static final int backLeftAbsoluteEncoderID = 0;
    public static final double backLeftAbsoluteEncoderOffset = 0.0;
    public static final boolean backLeftInvertAbsoluteEncoder = false;

    // * Back Left

    // * Back Right

    public static final int backRightDriveMotorID = 0;
    public static final MotorType backRightDriveMotorType = MotorType.kBrushless;
    public static final SparkBaseConfig backRightDriveMotorConfig = new SparkMaxConfig()
    .idleMode(null)
    .inverted(false)
    .apply(ModuleConstants.driveMotorDefaultConfig);
    public static final boolean backRightInvertDriveEncoder = false;

    public static final int backRightAngleMotorID = 0;
    public static final MotorType backRightAngleMotorType = MotorType.kBrushless;
    public static final SparkBaseConfig backRightAngleMotorConfig = new SparkMaxConfig()
    .idleMode(null)
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


    // TODO: Should we put these values in cm or m?
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

    public static final double robotMass = 0.0;
    public static final double robotMomentOfInertia = 0.0;

    public static final RobotConfig config = new RobotConfig(
      robotMass, 
      robotMomentOfInertia, 
      ModuleConstants.moduleConfig, 
      new Translation2d(DrivetrainConstants.moduleLateralDistance / 2, DrivetrainConstants.moduleLongitudinalDistance / 2), // Front Left
      new Translation2d(-DrivetrainConstants.moduleLateralDistance / 2, DrivetrainConstants.moduleLongitudinalDistance / 2), // Front Right
      new Translation2d(DrivetrainConstants.moduleLateralDistance / 2, -DrivetrainConstants.moduleLongitudinalDistance / 2), // Back Left
      new Translation2d(-DrivetrainConstants.moduleLateralDistance / 2, -DrivetrainConstants.moduleLongitudinalDistance / 2) // Back Right
    );
  }

  public static class VisionConstants {
    public static final String cameraName = "cam0";

    public static final Transform3d robotToCamTransform = new Transform3d();

    public static final PoseStrategy poseEstimationStrategy = PoseStrategy.LOWEST_AMBIGUITY;

    // TODO: Change this layout to the updated one.
    public static final AprilTagFieldLayout fieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2024Crescendo);
  }

  public static class AutoConstants {
    public static final double aprilTagDriveReductionFactor = 0.0;
    public static final double aprilTagAimReductionFactor = 0.0;
    public static final double aprilTagAngleTolerance = 0.0;
    public static final double aprilTagDistanceTolerance = 0.0;
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