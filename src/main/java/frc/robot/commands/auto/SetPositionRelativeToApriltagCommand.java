// * Unfinished
package frc.robot.commands.auto;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.AutoConstants;
import frc.robot.subsystems.swerve.DrivetrainSubsystem;
import frc.robot.units.VisionProcessingUnit;
import frc.robot.utils.DriveType;

public class SetPositionRelativeToApriltagCommand extends Command {

  DrivetrainSubsystem drivetrainSubsystem;

  VisionProcessingUnit vision = VisionProcessingUnit.getInstance();

  double xSpeed, ySpeed, rSpeed;

  int targetId;

  double desiredXOffset, currentXOffset;
  double desiredYOffset, currentYOffset;
  double desiredAngle, currentAngle;
  double xError, yError, angleError;

  public SetPositionRelativeToApriltagCommand(
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

  }

  @Override
  public void execute() {
  
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
