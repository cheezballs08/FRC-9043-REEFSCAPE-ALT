package frc.robot.subsystems.drivetrain;

import java.io.File;
import java.io.IOException;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.RobotConstants;
import frc.robot.utils.DriveType;
import frc.robot.utils.Logger;
import swervelib.SwerveDrive;
import swervelib.parser.SwerveParser;
import swervelib.telemetry.SwerveDriveTelemetry;
import swervelib.telemetry.SwerveDriveTelemetry.TelemetryVerbosity;

public class DefaultSwerve extends SubsystemBase implements DrivetrainSubsystem {
  
  SwerveDrive swerveDrive;

  public DefaultSwerve() {
    
    try {
      this.swerveDrive = new SwerveParser(new File(Filesystem.getDeployDirectory(), "swerve"))
      .createSwerveDrive(5);
    
    } catch (IOException e) {
      e.printStackTrace();
    }
    
    SwerveDriveTelemetry.verbosity = TelemetryVerbosity.NONE;

    this.resetOdometry(RobotConstants.initialPose);

    swerveDrive.setCosineCompensator(false);
    swerveDrive.setHeadingCorrection(false);
  }
  
  @Override
  public void periodic() {
    this.updateOdometer();

    Logger.log("Drivetrain/SimPose", this.getSimPose());
    Logger.log("Drivetrain/OdometryPose", this.getPose());

    Logger.log("Drivetrain/FieldRelativeSpeeds", this.getFieldRelativeSpeeds());
    Logger.log("Drivetrain/RobotRelativeSpeeds", this.getRobotRelativeSpeeds());

    Logger.log("Drivetrain/SwerveModuleStates", swerveDrive.getStates());
  }

  public ChassisSpeeds getFieldRelativeSpeeds() {
    return swerveDrive.getFieldVelocity();
  }

  public ChassisSpeeds getRobotRelativeSpeeds() {
    return swerveDrive.getRobotVelocity();
  }

  public void resetOdometry(Pose2d pose) {
    swerveDrive.resetOdometry(pose);
  }

  public void stop() {
    swerveDrive.drive(new ChassisSpeeds());
  }

  public void drive(ChassisSpeeds speeds) {
    swerveDrive.drive(speeds);
  }

  public void drive(double xSpeed, double ySpeed, double rSpeed, DriveType driveType) {
    if (driveType == DriveType.FieldRelative) {
      swerveDrive.driveFieldOriented(new ChassisSpeeds(xSpeed, ySpeed, rSpeed));
    
    } else {
      swerveDrive.drive(new ChassisSpeeds(xSpeed, ySpeed, rSpeed));  
    }
  }

/*  public void drive(ChassisSpeeds speeds, DriveFeedforwards feedforwards) {
    DogLog.log("/Dirty/Speeds", speeds);
    DogLog.log("/Dirty/Accels", feedforwards.accelerationsMPSSq());
    DogLog.log("/Dirty/Forces", feedforwards.linearForces().toString());
    
    swerveDrive.drive(
      speeds,
      swerveDrive.kinematics.toSwerveModuleStates(speeds),
      feedforwards.linearForces()
      );
  }*/

  public Pose2d getPose() {
    return swerveDrive.getPose();
  }

  public Pose2d getSimPose() {
    return swerveDrive.getSimulationDriveTrainPose().get();
  }

  public void updateOdometer() {
    swerveDrive.updateOdometry();
  }
}
