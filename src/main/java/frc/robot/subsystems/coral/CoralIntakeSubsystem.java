package frc.robot.subsystems.coral;

import org.littletonrobotics.junction.mechanism.LoggedMechanismLigament2d;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public interface CoralIntakeSubsystem extends Subsystem {

  public void setSpeeds(double speed);

  public void setAngle(double angle);

  public boolean isAtSetpoint();

  public boolean isSensorActive();

  public Trigger getSensorAsTrigger();

  public LoggedMechanismLigament2d getLigament();

  public Pose3d getPose();
}
