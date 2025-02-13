package frc.robot.subsystems.elevator;

import edu.wpi.first.wpilibj2.command.Subsystem;

public interface ElevatorSubsystem extends Subsystem {

  void setElevatorPosition(double desiredPosition);
  
  public void setSpeeds(double speed);

  public void setVoltages(double voltage);
}
