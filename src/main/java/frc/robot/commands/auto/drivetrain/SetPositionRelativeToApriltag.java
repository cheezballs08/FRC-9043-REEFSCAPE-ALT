// Şuan işlem altında, ne yapacağımı biliyorum sadece yetiştiremedim :(

package frc.robot.commands.auto.drivetrain;

import org.photonvision.targeting.PhotonTrackedTarget;

import frc.robot.utils.Logger;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.AutoConstants;
import frc.robot.constants.VisionConstants;
import frc.robot.subsystems.drivetrain.DrivetrainSubsystem;
import frc.robot.units.VisionProcessingUnit;
import frc.robot.utils.CameraPosition;
import frc.robot.utils.DriveType;

public class SetPositionRelativeToApriltag extends Command {

  DrivetrainSubsystem drivetrainSubsystem;

  VisionProcessingUnit frontUnit = VisionProcessingUnit.getUnit(CameraPosition.Front);

  double xSpeed, ySpeed, rSpeed;

  int targetId;

  PhotonTrackedTarget target;

  double desiredXOffset, currentXOffset;
  double desiredYOffset, currentYOffset;
  double desiredAngle, currentAngle;
  double xError, yError, angleError;

  Transform3d targetToCamera;
  Transform3d robotToTarget;

  public SetPositionRelativeToApriltag(
    DrivetrainSubsystem drivetrainSubsystem,
    int targetId,
    double desiredXOffset,
    double desiredYOffset,
    double desiredAngle
    ) {

    this.drivetrainSubsystem = drivetrainSubsystem;

    this.targetId = targetId;

    this.desiredXOffset = desiredXOffset;
    this.desiredYOffset = desiredYOffset;
    this.desiredAngle = desiredAngle;

    addRequirements(drivetrainSubsystem);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {

    if (!frontUnit.isSeen(targetId)) {
      return;
    }

    target = frontUnit.getTarget(20);

    targetToCamera = target.getBestCameraToTarget();

    robotToTarget = VisionConstants.robotToFrontCameraTransform.inverse().plus(targetToCamera); 

    currentXOffset = robotToTarget.getX();
    currentYOffset = robotToTarget.getY();
    currentAngle = target.getYaw();

    Logger.log("CurX", currentXOffset);
    Logger.log("CurY", currentYOffset);
    Logger.log("CurAngle", currentAngle);

    // TODO: Deneme yanılmayla bunları düzelt.
    xError = desiredXOffset - currentXOffset;
    yError = desiredYOffset - currentYOffset;
    angleError = desiredAngle - currentAngle;

    Logger.log("xError", xError);
    Logger.log("yError", yError);
    Logger.log("angleError", angleError);
    

    xSpeed = - xError;
    ySpeed = - yError;
    rSpeed = Math.toRadians(angleError) * 2;

    drivetrainSubsystem.drive(xSpeed, ySpeed, rSpeed, DriveType.RobotRelative);
  }

  @Override
  public void end(boolean interrupted) {
    drivetrainSubsystem.stop();
  }

  @Override
  public boolean isFinished() {
    if (
      Math.abs(xError) < AutoConstants.aprilTagDistanceTolerance
      && 
      Math.abs(yError) < AutoConstants.aprilTagDistanceTolerance
      &&
      Math.abs(angleError) < AutoConstants.aprilTagAngleTolerance) {
      return true;
    }
    return false;
  }
}
