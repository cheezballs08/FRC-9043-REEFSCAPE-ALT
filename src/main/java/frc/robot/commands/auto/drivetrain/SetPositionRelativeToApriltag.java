package frc.robot.commands.auto.drivetrain;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.constants.AutoConstants;
import frc.robot.utils.DrivetrainSubsystem;
import frc.robot.units.VisionProcessingUnit;
import frc.robot.utils.CameraPosition;
import frc.robot.utils.DriveType;

public class SetPositionRelativeToApriltag extends Command {

  DrivetrainSubsystem drivetrainSubsystem;

  VisionProcessingUnit vision = VisionProcessingUnit.getUnit(CameraPosition.Front);

  double xSpeed, ySpeed, rSpeed;

  int targetId;

  double desiredXOffset, currentXOffset;
  double desiredYOffset, currentYOffset;
  double desiredAngle, currentAngle;
  double xError, yError, angleError;

  Pose3d targetPose;
  Pose3d cameraPose;
  

  public SetPositionRelativeToApriltag(
    DrivetrainSubsystem drivetrainSubsystem,
    DriveType driveType,
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
  public void initialize() {
    if (!vision.isSeen(targetId)) {
      CommandScheduler.getInstance().cancel(this);
    }
  }

  @Override
  public void execute() {

    // TODO: Robot yüksekliği önemli mi?
    cameraPose = new Pose3d(drivetrainSubsystem.getPose());

    targetPose = cameraPose.transformBy(vision.getTarget(targetId).getBestCameraToTarget());

    currentXOffset = targetPose.getX();
    currentYOffset = targetPose.getY();
    currentAngle = targetPose.getRotation().getAngle();

    // TODO: Deneme yanılmayla bunları düzelt.
    xError = desiredXOffset - currentXOffset;
    yError = desiredYOffset - currentYOffset;
    angleError = desiredAngle - currentAngle;

    drivetrainSubsystem.drive(xSpeed, ySpeed, rSpeed, DriveType.FieldRelative);
  }

  @Override
  public void end(boolean interrupted) {
    drivetrainSubsystem.stop();
  }

  @Override
  public boolean isFinished() {
    if (
      Math.abs(xError) < AutoConstants.aprilTagDistanceTolerance
      || 
      Math.abs(yError) < AutoConstants.aprilTagDistanceTolerance
      ||
      Math.abs(angleError) < AutoConstants.aprilTagAngleTolerance 
      ||
      !vision.isSeen(targetId)) {
      return true;
    }
    return false;
  }
}
