package frc.robot.commands.teleop.algea;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.AlgaeIntakeSubsystem;

// TODO: Brake yeterli mi değil mi kontrol et ve gerekirse hız ver.
public class AlgeaIntakeIdleCommand extends Command {
  
  AlgaeIntakeSubsystem subsystem;

  public AlgeaIntakeIdleCommand(AlgaeIntakeSubsystem subsystem) {
    this.subsystem = subsystem;
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {}

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    return false;
  }
}
