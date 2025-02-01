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
import frc.robot.constants.ModuleConstants;
import frc.robot.constants.RobotConstants;
import frc.robot.units.VisionProcessingUnit;
import frc.robot.utils.DriveType;
import frc.robot.utils.Gyroscope;
import frc.robot.constants.DrivetrainConstants;

public class DrivetrainSubsystem extends SubsystemBase {
  
  private SwerveModule frontLeftModule = new SwerveModule(
    DrivetrainConstants.frontLeftDriveMotorID, 
    DrivetrainConstants.frontLeftDriveMotorConfig, 
    DrivetrainConstants.frontLeftInvertDriveEncoder, 
    DrivetrainConstants.frontLeftAngleMotorID, 
    DrivetrainConstants.frontLeftAngleMotorConfig, 
    DrivetrainConstants.frontLeftInvertAngleEncoder, 
    DrivetrainConstants.frontLeftAbsoluteEncoderID, 
    DrivetrainConstants.frontLeftAbsoluteEncoderOffset, 
    DrivetrainConstants.frontLeftInvertAbsoluteEncoder
  );
  
  private SwerveModule frontRightModule = new SwerveModule(
    DrivetrainConstants.frontRightDriveMotorID, 
    DrivetrainConstants.frontRightDriveMotorConfig, 
    DrivetrainConstants.frontRightInvertDriveEncoder, 
    DrivetrainConstants.frontRightAngleMotorID, 
    DrivetrainConstants.frontRightAngleMotorConfig, 
    DrivetrainConstants.frontRightInvertAngleEncoder, 
    DrivetrainConstants.frontRightAbsoluteEncoderID, 
    DrivetrainConstants.frontRightAbsoluteEncoderOffset, 
    DrivetrainConstants.frontRightInvertAbsoluteEncoder
  );
  
  private SwerveModule backLeftModule = new SwerveModule(
    DrivetrainConstants.backLeftDriveMotorID, 
    DrivetrainConstants.backLeftDriveMotorConfig, 
    DrivetrainConstants.backLeftInvertDriveEncoder, 
    DrivetrainConstants.backLeftAngleMotorID, 
    DrivetrainConstants.backLeftAngleMotorConfig, 
    DrivetrainConstants.backLeftInvertAngleEncoder, 
    DrivetrainConstants.backLeftAbsoluteEncoderID,  
    DrivetrainConstants.backLeftAbsoluteEncoderOffset, 
    DrivetrainConstants.backLeftInvertAbsoluteEncoder
  );

  private SwerveModule backRightModule = new SwerveModule(
    DrivetrainConstants.backRightDriveMotorID, 
    DrivetrainConstants.backRightDriveMotorConfig,
    DrivetrainConstants.backRightInvertDriveEncoder, 
    DrivetrainConstants.backRightAngleMotorID, 
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
    RobotConstants.initialPose
    );

  public DrivetrainSubsystem() {
    // For reseting the gyro.
    new Thread( 
    () -> {
      try {
        while (gyroscope.isCalibrating()) {
          Thread.sleep(10);
        }
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
      // TODO: Bunu feedforwardları da kullanacak şekilde ayarla.
      (speeds, feedworwards) -> this.drive(speeds), 
      controller, 
      RobotConstants.config, 
      () -> DriverStation.getAlliance().isPresent() && DriverStation.getAlliance().get() == DriverStation.Alliance.Red, 
      this
    );
  }

  @Override
  public void periodic() {
  }

  public void updateOdometer() {

    if (vision.canEstimatePose()) {

      // TODO: timestampSeconds değerinin bu iş için uygun olup olmadığına bak.
      // değişmediyse bildiğim kadarıyla yanlış, addVisionMeasurement ın üstüne gidinceki açılamada anlatıyor ne kullanman gerektiğini ama bildiğim kadarıyla
      //senin yaptığın vision başladığından beri bir süre, ihtiyacın olan roborio yazılımı başladığından beri geçen süre.
      odometer.addVisionMeasurement(vision.getEstimatedPose2d(), vision.getEstimate().timestampSeconds);
    }

    odometer.update(this.getRotation2d(), this.getModulePositions());

  }

  public Pose2d getPose() {
    return odometer.getEstimatedPosition();
  }

  public double getRobotAngle() {
    // TODO: Bunu yapmalımıyım yoksa otomatik olarak yapılıyor mu?
    // navxin getangle ı accumilated yani 360tan öteye gidiyor, illa ieeeremainder ile yapmana gerek yok modülüs (%) ile de olur ama hangisi daha hızlı ona emin değilim.
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


  // TODO: Bu iki metod baya yanlış olabilir. Kontrol et.
  // doğru gibi ama wpilib dokümanlarından bakmakta fayda var
  public ChassisSpeeds getFieldRelativeSpeeds() {
    return DrivetrainConstants.kinematics.toChassisSpeeds(this.getModuleStates());
  }

  public ChassisSpeeds getRobotRelativeSpeeds() {
    return ChassisSpeeds.fromFieldRelativeSpeeds(this.getFieldRelativeSpeeds(), this.getRotation2d());
  }

  // TODO: Bu böyle doğru mu kontrol et.
  // okay gibi duruyor
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
