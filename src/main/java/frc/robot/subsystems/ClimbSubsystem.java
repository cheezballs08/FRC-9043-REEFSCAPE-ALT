package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import frc.robot.constants.MotorConstants;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ClimbSubsystem extends SubsystemBase {

  SparkMax motor;

  // Sabitler

  int motorId = 0;

  MotorType motorType = null;

  SparkBaseConfig motorDefaultConfig = new SparkMaxConfig()
  .smartCurrentLimit(0)
  .idleMode(null);

  SparkBaseConfig motorConfig = new SparkMaxConfig()
  .apply(motorDefaultConfig)
  .inverted(false);

  // Sabitler bitti

  public ClimbSubsystem() {
    this.motor = new SparkMax(motorId, motorType);
    this.motor.configure(motorConfig, MotorConstants.resetMode, MotorConstants.persistMode);
  }

  //Motor hızı ayarlama
  public void setMotorSpeed(double speed){
    this.motor.set(speed);
  }

  @Override
  public void periodic() {

  }
}
