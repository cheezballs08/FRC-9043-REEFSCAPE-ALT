// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.swerve;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.studica.frc.AHRS;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ModuleConstants;
import frc.robot.Constants.RobotConsants;
import frc.robot.units.VisionProcessingUnit;
import frc.robot.utils.DriveType;
import frc.robot.utils.Gyroscope;
import frc.robot.Constants.DrivetrainConstants;

public class DrivetrainSubsystem extends SubsystemBase {
  
  private SwerveModule frontLeftModule = new SwerveModule(
    DrivetrainConstants.frontLeftDriveMotorID, 
    DrivetrainConstants.frontLeftDriveMotorType, 
    DrivetrainConstants.frontLeftDriveMotorConfig, 
    DrivetrainConstants.frontLeftInvertDriveEncoder, 
    DrivetrainConstants.frontLeftAngleMotorID, 
    DrivetrainConstants.frontLeftAngleMotorType, 
    DrivetrainConstants.frontLeftAngleMotorConfig, 
    DrivetrainConstants.frontLeftInvertAngleEncoder, 
    DrivetrainConstants.frontLeftAbsoluteEncoderID, 
    DrivetrainConstants.frontLeftAbsoluteEncoderOffset, 
    DrivetrainConstants.frontLeftInvertAbsoluteEncoder
  );
  
  private SwerveModule frontRightModule = new SwerveModule(
    DrivetrainConstants.frontRightDriveMotorID, 
    DrivetrainConstants.frontRightDriveMotorType, 
    DrivetrainConstants.frontRightDriveMotorConfig, 
    DrivetrainConstants.frontRightInvertDriveEncoder, 
    DrivetrainConstants.frontRightAngleMotorID, 
    DrivetrainConstants.frontRightAngleMotorType, 
    DrivetrainConstants.frontRightAngleMotorConfig, 
    DrivetrainConstants.frontRightInvertAngleEncoder, 
    DrivetrainConstants.frontRightAbsoluteEncoderID, 
    DrivetrainConstants.frontRightAbsoluteEncoderOffset, 
    DrivetrainConstants.frontRightInvertAbsoluteEncoder
  );
  
  private SwerveModule backLeftModule = new SwerveModule(
    DrivetrainConstants.backLeftDriveMotorID, 
    DrivetrainConstants.backLeftDriveMotorType, 
    DrivetrainConstants.backLeftDriveMotorConfig, 
    DrivetrainConstants.backLeftInvertDriveEncoder, 
    DrivetrainConstants.backLeftAngleMotorID, 
    DrivetrainConstants.backLeftAngleMotorType, 
    DrivetrainConstants.backLeftAngleMotorConfig, 
    DrivetrainConstants.backLeftInvertAngleEncoder, 
    DrivetrainConstants.backLeftAbsoluteEncoderID,  
    DrivetrainConstants.backLeftAbsoluteEncoderOffset, 
    DrivetrainConstants.backLeftInvertAbsoluteEncoder
  );

  private SwerveModule backRightModule = new SwerveModule(
    DrivetrainConstants.backRightDriveMotorID, 
    DrivetrainConstants.backRightDriveMotorType, 
    DrivetrainConstants.backRightDriveMotorConfig,
    DrivetrainConstants.backRightInvertDriveEncoder, 
    DrivetrainConstants.backRightAngleMotorID, 
    DrivetrainConstants.backRightAngleMotorType, 
    DrivetrainConstants.backRightAngleMotorConfig, 
    DrivetrainConstants.backRightInvertAngleEncoder, 
    DrivetrainConstants.backRightAbsoluteEncoderID, 
    DrivetrainConstants.backRightAbsoluteEncoderOffset, 
    DrivetrainConstants.backRightInvertAbsoluteEncoder
  );

  private Gyroscope gyroscope = new Gyroscope(new AHRS(DrivetrainConstants.GyroscopeCommunicationType), DrivetrainConstants.invertGyroscope);
  
  private PPHolonomicDriveController controller = new PPHolonomicDriveController(ModuleConstants.drivePID, ModuleConstants.anglePID);

  private VisionProcessingUnit vision = VisionProcessingUnit.getInstance();

  SwerveDrivePoseEstimator odometer = new SwerveDrivePoseEstimator(
    DrivetrainConstants.kinematics, 
    this.getRotation2d(), 
    this.getModulePositions(), 
    RobotConsants.initialPose
    );

  public DrivetrainSubsystem() {
    // For reseting the gyro.
    // TODO: Use isCalibrating for the thread duration.
    new Thread( 
    () -> {
      try {
        Thread.sleep(1000);
        this.resetRobotAngle();

      } catch (Exception e) {
        e.printStackTrace();
      }

    }
    ).start();

    // Configuring the autobuilder
    AutoBuilder.configure(
      this::getPose, 
      this::resetOdometry, 
      this::getRobotRelativeSpeeds, 
      // TODO: Make this use the feedforwards too.
      (speeds, feedworwards) -> this.drive(speeds), 
      controller, 
      RobotConsants.config, 
      () -> DriverStation.getAlliance().isPresent() && DriverStation.getAlliance().get() == DriverStation.Alliance.Red, 
      this
    );
  }

  @Override
  public void periodic() {
  }

  public void updateOdometer() {

    if (vision.canEstimatePose()) {

      // TODO: Check if timestampSeconds is correct for this job.
      odometer.addVisionMeasurement(vision.getEstimatedPose2d(), vision.getEstimate().timestampSeconds);
    }

    odometer.update(this.getRotation2d(), this.getModulePositions());

  }

  public Pose2d getPose() {
    return odometer.getEstimatedPosition();
  }

  public double getRobotAngle() {
    // TODO: Check if I need to do this or if it does this by itself
    return Math.IEEEremainder(gyroscope.getAngle(), 360);
  }

  public Rotation2d getRotation2d() {
    return Rotation2d.fromDegrees(this.getRobotAngle());
  }

  public SwerveModulePosition[] getModulePositions() {
    return new SwerveModulePosition[] {
      frontLeftModule.getPosition(),
      frontRightModule.getPosition(),
      backLeftModule.getPosition(),
      backRightModule.getPosition()
    };
  }


  // TODO: These two might be horribly wrong.
  public ChassisSpeeds getFieldRelativeSpeeds() {
    return DrivetrainConstants.kinematics.toChassisSpeeds(this.getModuleStates());
  }

  public ChassisSpeeds getRobotRelativeSpeeds() {
    return ChassisSpeeds.fromFieldRelativeSpeeds(this.getFieldRelativeSpeeds(), this.getRotation2d());
  }

  // TODO: Check if this is correct.
  public void resetOdometry(Pose2d pose) {
    odometer.resetPosition(this.getRotation2d(), this.getModulePositions(), pose);
  }

  public void resetRobotAngle() {
    gyroscope.reset();
  }

  public void stop() {
    frontLeftModule.stop();
    frontRightModule.stop();
    backLeftModule.stop();
    backRightModule.stop();
  }

  // Input in Front Left -> FrontRight -> BackLeft -> BackRight order
  public void setModuleStates(SwerveModuleState[] desiredStates) {
    SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, ModuleConstants.driveMaxSpeed);
    
    frontLeftModule.setDesiredState(desiredStates[0]);
    frontRightModule.setDesiredState(desiredStates[1]);
    backLeftModule.setDesiredState(desiredStates[2]);
    backRightModule.setDesiredState(desiredStates[3]);
  }

  public SwerveModuleState[] getModuleStates() {
    return new SwerveModuleState[] {
      frontLeftModule.getState(),
      frontRightModule.getState(),
      backLeftModule.getState(),
      backRightModule.getState()
    };
  }

  public void drive(ChassisSpeeds speeds) {
    this.setModuleStates(DrivetrainConstants.kinematics.toSwerveModuleStates(speeds));
  }

  public void drive(double xSpeed, double ySpeed, double rSpeed, DriveType driveType) {
    
    ChassisSpeeds speeds;

    if (driveType == DriveType.FieldRelative) {
      speeds = ChassisSpeeds.fromRobotRelativeSpeeds(xSpeed, ySpeed, rSpeed, this.getRotation2d());
    } else {
      speeds = new ChassisSpeeds(xSpeed, ySpeed, rSpeed);
    }

    this.setModuleStates(DrivetrainConstants.kinematics.toSwerveModuleStates(speeds));
  }

}
