// Şuan işlem altında, ne yapacağımı biliyorum sadece yetiştiremedim :(

package frc.robot.commands.auto.drivetrain;

import java.lang.annotation.Target;

import org.photonvision.targeting.PhotonTrackedTarget;

import dev.doglog.DogLog;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.constants.AutoConstants;
import frc.robot.constants.VisionConstants;
import frc.robot.subsystems.drivetrain.DrivetrainSubsystem;
import frc.robot.units.VisionProcessingUnit;
import frc.robot.utils.CameraPosition;
import frc.robot.utils.DriveType;

public class ChaseApriltag extends Command {

  DrivetrainSubsystem drivetrainSubsystem;

  VisionProcessingUnit frontUnit = VisionProcessingUnit.getUnit(CameraPosition.Front);

  double xSpeed, ySpeed, rSpeed;

  int targetId;

  PhotonTrackedTarget target;

  ProfiledPIDController xController;
  ProfiledPIDController yController;
  ProfiledPIDController rController;

  Transform3d apriltagToGoal;

  public ChaseApriltag(
    DrivetrainSubsystem drivetrainSubsystem,
    int targetId,
    double desiredXOffset,
    double desiredYOffset,
    double desiredAngle
    ) {

    this.drivetrainSubsystem = drivetrainSubsystem;

    this.targetId = targetId;

    this.apriltagToGoal = new Transform3d(
      desiredXOffset, 
      desiredYOffset, 
      0,
      new Rotation3d(new Rotation2d(Units.degreesToRadians(desiredAngle)))
    );

    this.xController = new ProfiledPIDController(
      AutoConstants.PDrive, 
      AutoConstants.IDrive, 
      AutoConstants.DDrive, 
      AutoConstants.driveConstraints
    );
    this.xController.setIZone(AutoConstants.IZDrive);
    this.xController.setTolerance(AutoConstants.distanceTolerance);

    this.yController = new ProfiledPIDController(
      AutoConstants.PDrive, 
      AutoConstants.IDrive, 
      AutoConstants.DDrive, 
      AutoConstants.driveConstraints
    );
    this.yController.setIZone(AutoConstants.IZDrive);
    this.yController.setTolerance(AutoConstants.distanceTolerance);

    this.rController = new ProfiledPIDController(
      AutoConstants.PAngle, 
      AutoConstants.IAngle, 
      AutoConstants.DAngle, 
      AutoConstants.angleConstraints
    );
    this.rController.setIZone(AutoConstants.IZAngle);
    this.rController.enableContinuousInput(-Math.PI, Math.PI);
    this.rController.setTolerance(AutoConstants.angleTolerance);

    addRequirements(drivetrainSubsystem);
  }

  @Override
  public void initialize() {
    if (frontUnit.isSeen(targetId)) {
      target = frontUnit.getTarget(targetId);  
      return;
    }
    CommandScheduler.getInstance().cancel(this); 
  }

  @Override
  public void execute() {

    if (frontUnit.isSeen(targetId)) {
      target = frontUnit.getTarget(targetId);  
      
    } else {
      return;
    }

    Pose3d robotPose = new Pose3d(drivetrainSubsystem.getPose());
    DogLog.log("ChaseApriltag/RobotPose", robotPose);

    Transform3d camToTarget = target.getBestCameraToTarget();

    Pose3d cameraPose = robotPose.transformBy(VisionConstants.robotToFrontCameraTransform);
    DogLog.log("ChaseApriltag/CameraPose", cameraPose);

    Pose3d targetPose = cameraPose.transformBy(camToTarget);
    DogLog.log("ChaseApriltag/TargetPose", targetPose);

    Pose3d goalPose = targetPose.transformBy(apriltagToGoal);
    DogLog.log("ChaseApriltag/GoalPose", goalPose);


    xSpeed = xController.calculate(robotPose.getX(), goalPose.getX());
    DogLog.log("ChaseApriltag/XSpeed", xSpeed);
    DogLog.log("ChaseApriltag/XError", xController.getPositionError());

    ySpeed = yController.calculate(robotPose.getY(), goalPose.getY());
    DogLog.log("ChaseApriltag/YSpeed", ySpeed);
    DogLog.log("ChaseApriltag/YError", yController.getPositionError());

    rSpeed = rController.calculate(robotPose.getRotation().getAngle(), goalPose.getRotation().getAngle());
    DogLog.log("ChaseApriltag/RSpeed", rSpeed);
    DogLog.log("ChaseApriltag/RError", rController.getPositionError());

    drivetrainSubsystem.drive(xSpeed, ySpeed, rSpeed, DriveType.RobotRelative);
  }

  @Override
  public void end(boolean interrupted) {
    drivetrainSubsystem.stop();
  }

  @Override
  public boolean isFinished() {
    if (xController.atSetpoint() && yController.atSetpoint() && rController.atSetpoint()) {
      return true;
    }  
    
    return false;
  }
}