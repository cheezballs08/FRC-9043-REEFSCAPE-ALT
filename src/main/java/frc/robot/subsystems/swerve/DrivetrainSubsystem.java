// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.swerve;

import com.pathplanner.lib.auto.AutoBuilder;
import com.studica.frc.AHRS;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ModuleConsants;
import frc.robot.Constants.RobotConsants;
import frc.robot.Constants.DrivetrainConstants;

public class DrivetrainSubsystem extends SubsystemBase {
  
  SwerveModule frontLeftModule = new SwerveModule(
    DrivetrainConstants.frontLeftDriveMotorID, 
    DrivetrainConstants.frontLeftDriveMotorType, 
    DrivetrainConstants.frontLeftInvertDriveMotor, 
    DrivetrainConstants.frontLeftInvertDriveEncoder, 
    DrivetrainConstants.frontLeftAngleMotorID, 
    DrivetrainConstants.frontLeftAngleMotorType, 
    DrivetrainConstants.frontLeftInvertAngleMotor, 
    DrivetrainConstants.frontLeftInvertAngleEncoder, 
    DrivetrainConstants.frontLeftAbsoluteEncoderID, 
    DrivetrainConstants.frontLeftAbsoluteEncoderOffset, 
    DrivetrainConstants.frontLeftInvertAbsoluteEncoder
  );
  
  SwerveModule frontRightModule = new SwerveModule(
    DrivetrainConstants.frontRightDriveMotorID, 
    DrivetrainConstants.frontRightDriveMotorType, 
    DrivetrainConstants.frontRightInvertDriveMotor, 
    DrivetrainConstants.frontRightInvertDriveEncoder, 
    DrivetrainConstants.frontRightAngleMotorID, 
    DrivetrainConstants.frontRightAngleMotorType, 
    DrivetrainConstants.frontRightInvertAngleMotor, 
    DrivetrainConstants.frontRightInvertAngleEncoder, 
    DrivetrainConstants.frontRightAbsoluteEncoderID, 
    DrivetrainConstants.frontRightAbsoluteEncoderOffset, 
    DrivetrainConstants.frontRightInvertAbsoluteEncoder
  );
  
  SwerveModule backLeftModule = new SwerveModule(
    DrivetrainConstants.backLeftDriveMotorID, 
    DrivetrainConstants.backLeftDriveMotorType, 
    DrivetrainConstants.backLeftInvertDriveMotor, 
    DrivetrainConstants.backLeftInvertDriveEncoder, 
    DrivetrainConstants.backLeftAngleMotorID, 
    DrivetrainConstants.backLeftAngleMotorType, 
    DrivetrainConstants.backLeftInvertAngleMotor, 
    DrivetrainConstants.backLeftInvertAngleEncoder, 
    DrivetrainConstants.backLeftAbsoluteEncoderID,  
    DrivetrainConstants.backLeftAbsoluteEncoderOffset, 
    DrivetrainConstants.backLeftInvertAbsoluteEncoder
  );

  SwerveModule backRightModule = new SwerveModule(
    DrivetrainConstants.backRightDriveMotorID, 
    DrivetrainConstants.backRightDriveMotorType, 
    DrivetrainConstants.backRightInvertDriveMotor, 
    DrivetrainConstants.backRightInvertDriveEncoder, 
    DrivetrainConstants.backRightAngleMotorID, 
    DrivetrainConstants.backRightAngleMotorType, 
    DrivetrainConstants.backRightInvertAngleMotor, 
    DrivetrainConstants.backRightInvertAngleEncoder, 
    DrivetrainConstants.backRightAbsoluteEncoderID, 
    DrivetrainConstants.backRightAbsoluteEncoderOffset, 
    DrivetrainConstants.backRightInvertAbsoluteEncoder
  );

  AHRS gyroscope = new AHRS(DrivetrainConstants.GyroscopeCommunicationType);

  // TODO: Swap this for the SwerveDrivePoseEstimator which uses the vision as well.
  SwerveDriveOdometry odometer = new SwerveDriveOdometry(
    DrivetrainConstants.kinematics, 
    this.getRotation2d(), 
    this.getModulePositions(), 
    RobotConsants.initialPose
    );

  public DrivetrainSubsystem() {
    // For reseting the gyro.
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

    // TODO: Configure this.
    AutoBuilder.configure(
      this::getPose, 
      this::resetOdometry, 
      null, 
      null, 
      null, 
      null, 
      null, 
      null
    );
  }

  @Override
  public void periodic() {
    odometer.update(this.getRotation2d(), this.getModulePositions());
  }

  public Pose2d getPose() {
    return odometer.getPoseMeters();
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

  // TODO: Check if this is correct.
  public void resetOdometry(Pose2d pose) {
    odometer.resetPosition(getRotation2d(), getModulePositions(), pose);
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
    SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, ModuleConsants.driveMaxSpeed);
    
    frontLeftModule.setDesiredState(desiredStates[0]);
    frontRightModule.setDesiredState(desiredStates[1]);
    backLeftModule.setDesiredState(desiredStates[2]);
    backRightModule.setDesiredState(desiredStates[3]);
  }


  public void drive(ChassisSpeeds speeds) {
    this.setModuleStates(DrivetrainConstants.kinematics.toSwerveModuleStates(speeds));
  }

}
