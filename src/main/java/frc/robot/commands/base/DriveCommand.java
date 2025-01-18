package frc.robot.commands.base;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.swerve.DrivetrainSubsystem;
import frc.robot.utils.DriveType;

public class DriveCommand extends Command {

  DrivetrainSubsystem drivetrainSubsystem;
 
  DriveType driveType;

  Supplier<Double> xInputSupplier, yInputSupplier, rInputSupplier;

  double xInput, yInput, rInput;

  public DriveCommand(
    DrivetrainSubsystem drivetrainSubsystem,
    DriveType driveType,
    Supplier<Double> xInputSupplier,
    Supplier<Double> yInputSupplier,
    Supplier<Double> rInputSupplier  
  ) {

    this.drivetrainSubsystem = drivetrainSubsystem;
    
    this.driveType = driveType;
    
    this.xInputSupplier = xInputSupplier;
    this.yInputSupplier = yInputSupplier;
    this.rInputSupplier = rInputSupplier;

    addRequirements(drivetrainSubsystem);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    xInput = xInputSupplier.get();
    yInput = yInputSupplier.get();
    rInput = rInputSupplier.get();

    drivetrainSubsystem.drive(yInput, xInput, rInput, driveType);
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
