// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.teleop;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.AlgaeIntakeSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class AlgaeIntakeCommand extends Command {
  /** Creates a new AlgaeIntakeCommand. */

  AlgaeIntakeSubsystem subsystem;
  boolean isIntake = true;
  double startTime, curTime;

  //Constants
  int AlgaeMotorsIntakeTime = 1;
  double speed = 0.6;

  public AlgaeIntakeCommand(AlgaeIntakeSubsystem subsystem) {
    this.subsystem = subsystem;

    addRequirements(this.subsystem);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    startTime = System.currentTimeMillis();
    subsystem.SetAlgaeIntakeMotorsRotation(speed, isIntake);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    curTime = System.currentTimeMillis();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    subsystem.SetAlgaeIntakeMotorsRotation(0, isIntake);
    isIntake = !isIntake;
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if(curTime - startTime >= AlgaeMotorsIntakeTime * 1000){
      return true;
    }
    return false;
  }
}
