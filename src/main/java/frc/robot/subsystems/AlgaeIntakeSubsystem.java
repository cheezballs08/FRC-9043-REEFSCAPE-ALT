package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.AlgeaIntakeConstants;
import frc.robot.constants.MotorConstants;

public class AlgaeIntakeSubsystem extends SubsystemBase {
  
  SparkMax motor1, motor2;

  public AlgaeIntakeSubsystem() {
    this.motor1 = new SparkMax(AlgeaIntakeConstants.motor1ID, AlgeaIntakeConstants.motor1Type);
    this.motor1.configure(AlgeaIntakeConstants.motor1Config, MotorConstants.resetMode, MotorConstants.persistMode);

    this.motor2 = new SparkMax(AlgeaIntakeConstants.motor2ID, AlgeaIntakeConstants.motor2Type);
    this.motor2.configure(AlgeaIntakeConstants.motor1Config, MotorConstants.resetMode, MotorConstants.persistMode);
  }

  @Override
  public void periodic() {}

  public void setMotorSpeeds(double speed){
    motor1.set(speed);
    motor2.set(speed);
  }

  public void setMotorRotation(double speed, boolean isIntake){
    if(!(isIntake)) speed *= -1; 
    setMotorSpeeds(speed);
  }

}
