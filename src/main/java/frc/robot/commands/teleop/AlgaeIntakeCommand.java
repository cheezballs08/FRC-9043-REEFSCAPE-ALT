package frc.robot.commands.teleop;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.AlgaeIntakeSubsystem;

public class AlgaeIntakeCommand extends Command {

  AlgaeIntakeSubsystem subsystem;
  
  boolean isIntake = true;
  
  double startTime, currentTime;

  // Sabitler
  int algaeMotorsIntakeTime = 1;

  double speed = 0.6;

  public AlgaeIntakeCommand(AlgaeIntakeSubsystem subsystem) {
    this.subsystem = subsystem;

    addRequirements(this.subsystem);
  }

  @Override
  public void initialize() {
    startTime = Timer.getTimestamp();
    subsystem.setAlgaeIntakeMotorsRotation(speed, isIntake);
  }

  @Override
  public void execute() {
    currentTime = Timer.getTimestamp();
  }

  @Override
  public void end(boolean interrupted) {
    subsystem.setAlgaeIntakeMotorsRotation(0, isIntake);
    isIntake = !isIntake;
  }

  @Override
  public boolean isFinished() {
    if(currentTime - startTime >= algaeMotorsIntakeTime * 1000){
      return true;
    }
    return false;
  }
}
