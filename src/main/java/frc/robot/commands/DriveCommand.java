package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ControllerConsants;
import frc.robot.subsystems.swerve.DrivetrainSubsystem;
import frc.robot.utils.DriveType;

public class DriveCommand extends Command {

  // 

  DrivetrainSubsystem drivetrainSubsystem;
 
  DriveType driveType;

  Supplier<Double> xInputSupplier, yInputSupplier, rInputSupplier;

  SlewRateLimiter xLimiter, yLimiter, rLimiter;

  // 

  double xInput, yInput, rInput;


  public DriveCommand(
    DrivetrainSubsystem swerveSubsystem,
    DriveType driveType,
    Supplier<Double> xInputSupplier,
    Supplier<Double> yInputSupplier,
    Supplier<Double> rInputSupplier  
  ) {

    this.drivetrainSubsystem = swerveSubsystem;
    
    this.driveType = driveType;
    
    this.xInputSupplier = xInputSupplier;
    this.xLimiter = new SlewRateLimiter(ControllerConsants.maxAllowedDriveAcceleration);
    
    this.yInputSupplier = yInputSupplier;
    this.yLimiter = new SlewRateLimiter(ControllerConsants.maxAllowedDriveAcceleration);

    this.rInputSupplier = rInputSupplier;
    this.rLimiter = new SlewRateLimiter(ControllerConsants.maxAllowedAngleAcceleration);

    addRequirements(swerveSubsystem);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    xInput = xInputSupplier.get();
    yInput = yInputSupplier.get();
    rInput = rInputSupplier.get(); 
  
    xInput = Math.abs(xInput) > ControllerConsants.deadband ? xInput : 0.0;
    yInput = Math.abs(yInput) > ControllerConsants.deadband ? yInput : 0.0;
    rInput = Math.abs(rInput) > ControllerConsants.deadband ? rInput : 0.0;

    xInput = xLimiter.calculate(xInput) * ControllerConsants.driveSpeedReductionFactor;
    yInput = yLimiter.calculate(yInput) * ControllerConsants.driveSpeedReductionFactor;
    rInput = rLimiter.calculate(rInput) * ControllerConsants.angleSpeedReductionFactor;

    if (driveType == DriveType.FieldRelative) {
      drivetrainSubsystem.drive(ChassisSpeeds.fromFieldRelativeSpeeds(xInput, yInput, rInput, drivetrainSubsystem.getRotation2d()));
    } else {
      drivetrainSubsystem.drive(ChassisSpeeds.fromRobotRelativeSpeeds(xInput, yInput, rInput, drivetrainSubsystem.getRotation2d()));
    }
  }

  @Override
  public void end(boolean interrupted) {
    drivetrainSubsystem.stop();
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
