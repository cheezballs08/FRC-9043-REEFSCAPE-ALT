package frc.robot.commands.teleop.climb;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ClimbSubsystem;

public class ClimbCommand extends Command {

  ClimbSubsystem subsystem;

  double motorSpeed = 0.5;
  boolean isReverse = false;

  public ClimbCommand(ClimbSubsystem subsystem, boolean isReverse) {

    this.subsystem = subsystem;

    this.isReverse = isReverse;

    addRequirements(this.subsystem);
  }

  @Override
  public void initialize() {
    // Eğer kıskaç açılma tuşuna basılı tutulursa motor rötasyonunu tersine döndürür.
    motorSpeed = isReverse ? motorSpeed : motorSpeed * -1;
    subsystem.setMotorSpeed(motorSpeed);
  }

  @Override
  public void execute() {}

  @Override
  public void end(boolean interrupted) {
    subsystem.setMotorSpeed(0);
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
