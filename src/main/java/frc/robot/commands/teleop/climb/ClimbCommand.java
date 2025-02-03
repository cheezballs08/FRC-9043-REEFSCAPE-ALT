package frc.robot.commands.teleop.climb;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ClimbSubsystem;

public class ClimbCommand extends Command {

  ClimbSubsystem subsystem;

  double speed = 0.0;

  public ClimbCommand(ClimbSubsystem subsystem, double speed) {

    this.subsystem = subsystem;

    this.speed = speed;

    addRequirements(this.subsystem);
  }

  @Override
  public void initialize() {
    subsystem.setMotorSpeed(speed);
  }

  @Override
  public void execute() {
    subsystem.setMotorSpeed(speed);
  }

  @Override
  public void end(boolean interrupted) {
    subsystem.setMotorSpeed(0);
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
