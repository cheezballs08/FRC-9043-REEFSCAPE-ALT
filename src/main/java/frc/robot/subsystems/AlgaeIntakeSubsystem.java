package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class AlgaeIntakeSubsystem extends SubsystemBase {
  
  SparkMax motor1, motor2;

  // Sabitler
  int motor1ID = 0, motor2ID = 0;
  
  MotorType motor1Type = null, motor2Type = null;

  SparkBaseConfig motorDefaultConfig = new SparkMaxConfig()
  .smartCurrentLimit(0)
  .idleMode(null);

  SparkBaseConfig motor1Config = new SparkMaxConfig()
  .apply(motorDefaultConfig)
  .inverted(false);

  SparkBaseConfig motor2Config = new SparkMaxConfig()
  .apply(motorDefaultConfig)
  .inverted(false);

  // TODO: Bu persist ve reset mode'u genel bir sabit sınıfına atarak her alt birimide aynı kullanılmasını sağla
  PersistMode persistMode = PersistMode.kPersistParameters;

  ResetMode resetMode = ResetMode.kNoResetSafeParameters;

  public AlgaeIntakeSubsystem() {
    this.motor1 = new SparkMax(motor1ID, motor1Type);
    this.motor1.configure(motor1Config, resetMode, persistMode);

    this.motor2 = new SparkMax(motor2ID, motor2Type);
    this.motor2.configure(motor1Config, resetMode, persistMode);
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
