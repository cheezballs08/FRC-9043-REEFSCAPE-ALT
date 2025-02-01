package frc.robot.commands.teleop;

import java.util.function.Supplier;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.constants.AutoConstants;
import frc.robot.constants.ControllerConstants;
import frc.robot.subsystems.swerve.DrivetrainSubsystem;
import frc.robot.units.VisionProcessingUnit;
import frc.robot.utils.DriveType;

public class AimAtAprilTagCommand extends Command {

  DrivetrainSubsystem drivetrainSubsystem;

  VisionProcessingUnit vision = VisionProcessingUnit.getInstance();
  
  DriveType driveType;

  Supplier<Double> xSpeedSupplier, ySpeedSupplier;
  
  SlewRateLimiter xLimiter, yLimiter, rLimiter;

  double xSpeed, ySpeed, rSpeed;

  int targetId;

  double angleError;

  public AimAtAprilTagCommand(
    DrivetrainSubsystem drivetrainSubsystem,
    DriveType driveType,
    Supplier<Double> xSpeedSupplier,
    Supplier<Double> ySpeedSupplier,
    int targetId
  ) {
    this.drivetrainSubsystem = drivetrainSubsystem;

    this.driveType = driveType;

    this.xSpeedSupplier = xSpeedSupplier;
    this.xLimiter = new SlewRateLimiter(ControllerConstants.maxAllowedDriveAcceleration);
    
    this.ySpeedSupplier = ySpeedSupplier;
    this.yLimiter = new SlewRateLimiter(ControllerConstants.maxAllowedDriveAcceleration);  

    this.rLimiter = new SlewRateLimiter(ControllerConstants.maxAllowedAngleAcceleration);

    this.targetId = targetId;

    addRequirements(drivetrainSubsystem);

  }

  public void updateSpeeds() {
    xSpeed = xSpeedSupplier.get();
    ySpeed = ySpeedSupplier.get();
    
    xSpeed = Math.abs(xSpeed) > ControllerConstants.deadband ? xSpeed : 0.0;
    ySpeed = Math.abs(ySpeed) > ControllerConstants.deadband ? ySpeed : 0.0;

    xSpeed = xLimiter.calculate(xSpeed) * ControllerConstants.driveSpeedReductionFactor;
    ySpeed = yLimiter.calculate(ySpeed) * ControllerConstants.driveSpeedReductionFactor;
    
    this.calculateRSpeed();
  }

  public void calculateRSpeed() {
    angleError = vision.getTarget(targetId).getYaw();

    rSpeed = rLimiter.calculate(angleError * AutoConstants.aprilTagAimReductionFactor);
  }

  @Override
  public void initialize() {
    // TODO: Bunu yapmakta sıkıntı var mıdır.
    if (Math.abs(angleError) < AutoConstants.aprilTagAngleTolerance || !vision.isSeen(targetId)) {
      // This specifically.
      CommandScheduler.getInstance().cancel(this);
    }
  }

  @Override
  public void execute() {
    this.updateSpeeds();

    drivetrainSubsystem.drive(xSpeed, ySpeed, rSpeed, driveType);
  }

  @Override
  public void end(boolean interrupted) {
    drivetrainSubsystem.stop();
  }

  @Override
  public boolean isFinished() {

    if (Math.abs(angleError) < AutoConstants.aprilTagAngleTolerance || !vision.isSeen(targetId)) {
      return true;
    }

    return false;
  }
}
