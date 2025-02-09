package frc.robot.subsystems.drivetrain;

import java.io.IOException;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.util.DriveFeedforwards;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.DrivetrainConstants;
import frc.robot.constants.ModuleConstants;
import frc.robot.constants.RobotConstants;
import frc.robot.units.VisionProcessingUnit;
import frc.robot.utils.CameraPosition;
import frc.robot.utils.DriveType;
import frc.robot.utils.DrivetrainSubsystem;
import swervelib.SwerveDrive;
import swervelib.parser.SwerveParser;

public class DefaultSwerve extends SubsystemBase implements DrivetrainSubsystem {
  
  SwerveDrive swerveDrive;

  VisionProcessingUnit frontUnit = VisionProcessingUnit.getUnit(CameraPosition.Front);
  VisionProcessingUnit leftUnit = VisionProcessingUnit.getUnit(CameraPosition.Left);
  VisionProcessingUnit rightUnit = VisionProcessingUnit.getUnit(CameraPosition.Right);
  
  private PPHolonomicDriveController controller = new PPHolonomicDriveController(ModuleConstants.drivePID, ModuleConstants.anglePID);

  public DefaultSwerve() {
    try {
      this.swerveDrive = new SwerveParser(DrivetrainConstants.jsonDirectory)
      .createSwerveDrive(ModuleConstants.driveMaxSpeed);
    
    } catch (IOException e) {
      e.printStackTrace();
    }

    this.resetOdometry(RobotConstants.initialPose);

    swerveDrive.setCosineCompensator(false);

    AutoBuilder.configure(
      this::getPose, 
      this::resetOdometry, 
      this::getRobotRelativeSpeeds, 
      // TODO: bunu feedforwardları da kullanack şekilde değiştir
      (speeds, feedworwards) -> this.drive(speeds), 
      controller, 
      RobotConstants.config, 
      () -> RobotConstants.alliance != DriverStation.Alliance.Blue, 
      this
    );
  }

  @Override
  public void periodic() {
    this.updateOdometer();
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

  // TODO: Bunun niye çalışmadığına bak.
  public void drive(ChassisSpeeds speeds, DriveFeedforwards feedforwards) {
    swerveDrive.drive(speeds, swerveDrive.getStates(), feedforwards.linearForces());
  }

  public Pose2d getPose() {
    return swerveDrive.getPose();
  }

  public Pose2d getSimPose() {
    return swerveDrive.getSimulationDriveTrainPose().get();
  }

  public void updateOdometer() {
    if (frontUnit.canEstimatePose()) {
      swerveDrive.addVisionMeasurement(frontUnit.getEstimatedPose2d(), frontUnit.getEstimate().timestampSeconds);
    }
    /*if (leftUnit.canEstimatePose()) {
      swerveDrive.addVisionMeasurement(leftUnit.getEstimatedPose2d(), leftUnit.getEstimate().timestampSeconds);
    }
    if (rightUnit.canEstimatePose()) {
      swerveDrive.addVisionMeasurement(rightUnit.getEstimatedPose2d(), rightUnit.getEstimate().timestampSeconds);
    }*/

    swerveDrive.updateOdometry();
  }
}
