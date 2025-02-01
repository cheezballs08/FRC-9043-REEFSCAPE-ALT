package frc.robot.commands.teleop.algea;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.AlgaeIntakeSubsystem;

public class AlgeaIntakeSetSpeedCommand extends Command {

  private AlgaeIntakeSubsystem subsystem;

  private double speed;

  public AlgeaIntakeSetSpeedCommand(AlgaeIntakeSubsystem subsystem, double speed) {
  
    this.subsystem = subsystem;
    this.speed = speed;
  
  }

  @Override
  public void initialize() {
    subsystem.setMotorSpeeds(speed);
  }

  @Override
  public void execute() {
    subsystem.setMotorSpeeds(speed);
  }

  @Override
  public void end(boolean interrupted) {
    subsystem.setMotorSpeeds(0);
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
