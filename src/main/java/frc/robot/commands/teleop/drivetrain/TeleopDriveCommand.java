// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.teleop;

import java.util.function.Supplier;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.ControllerConstants;
import frc.robot.subsystems.swerve.DrivetrainSubsystem;
import frc.robot.utils.DriveType;

public class TeleopDriveCommand extends Command {

  DrivetrainSubsystem drivetrainSubsystem;
  
  DriveType driveType;

  Supplier<Double> xSpeedSupplier, ySpeedSupplier, rSpeedSupplier;
  
  SlewRateLimiter xLimiter, yLimiter, rLimiter;

  double xSpeed, ySpeed, rSpeed;

  public TeleopDriveCommand(
    DrivetrainSubsystem drivetrainSubsystem,
    DriveType driveType,
    Supplier<Double> xSpeedSupplier,
    Supplier<Double> ySpeedSupplier,
    Supplier<Double> rSpeedSupplier  
  ) {
    this.drivetrainSubsystem = drivetrainSubsystem;

    this.driveType = driveType;

    this.xSpeedSupplier = xSpeedSupplier;
    this.xLimiter = new SlewRateLimiter(ControllerConstants.maxAllowedDriveAcceleration);
    
    this.ySpeedSupplier = ySpeedSupplier;
    this.yLimiter = new SlewRateLimiter(ControllerConstants.maxAllowedDriveAcceleration);  

    this.rSpeedSupplier = rSpeedSupplier;
    this.rLimiter = new SlewRateLimiter(ControllerConstants.maxAllowedAngleAcceleration);

    addRequirements(drivetrainSubsystem);

  }

  public void updateSpeeds() {
    // TODO: Girdinin karesi veya küpünu alarak girdinin reaktivitesini azalt. 
    xSpeed = xSpeedSupplier.get();
    ySpeed = ySpeedSupplier.get();
    rSpeed = rSpeedSupplier.get(); 
  
    xSpeed = Math.abs(xSpeed) > ControllerConstants.deadband ? xSpeed : 0.0;
    ySpeed = Math.abs(ySpeed) > ControllerConstants.deadband ? ySpeed : 0.0;
    rSpeed = Math.abs(rSpeed) > ControllerConstants.deadband ? rSpeed : 0.0;

    xSpeed = xLimiter.calculate(xSpeed) * ControllerConstants.driveSpeedReductionFactor;
    ySpeed = yLimiter.calculate(ySpeed) * ControllerConstants.driveSpeedReductionFactor;
    rSpeed = rLimiter.calculate(rSpeed) * ControllerConstants.angleSpeedReductionFactor;
  }

  @Override
  public void initialize() {}

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
    return false;
  }
}
