package frc.robot.subsystems.elevator;

import org.littletonrobotics.junction.mechanism.LoggedMechanismLigament2d;

import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj2.command.Subsystem;

public interface ElevatorSubsystem extends Subsystem {

  public void setElevatorPosition(double desiredPosition);

  public void setVoltages(double voltage);

  public LoggedMechanismLigament2d getLigament();

}
