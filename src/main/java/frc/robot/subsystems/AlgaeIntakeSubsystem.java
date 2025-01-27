package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class AlgaeIntakeSubsystem extends SubsystemBase {
  
  SparkMax algaeIntakeMotor1, algaeIntakeMotor2;

  // Sabitler
  int algaeIntakeMotor1ID = 0, algaeIntakeMotor2ID = 0;
  MotorType algaeIntakeMotor1Type = null, algaeIntakeMotor2Type = null;
  boolean algaeIntakeMotor1Inv = false, algaeIntakeMotor2Inv = false;
  SparkBaseConfig sparkBaseConfig;

  public AlgaeIntakeSubsystem() {
    this.algaeIntakeMotor1 = new SparkMax(algaeIntakeMotor1ID, algaeIntakeMotor1Type);
    this.algaeIntakeMotor1.configure(sparkBaseConfig.inverted(algaeIntakeMotor1Inv), null, null);
    this.algaeIntakeMotor2 = new SparkMax(algaeIntakeMotor2ID, algaeIntakeMotor2Type);
    this.algaeIntakeMotor2.configure(sparkBaseConfig.inverted(algaeIntakeMotor2Inv), null, null);
  }

  @Override
  public void periodic() {}

  public void setAlgaeIntakeMotorsSpeeds(double speed){
    algaeIntakeMotor1.set(speed);
    algaeIntakeMotor2.set(speed);
  }

  public void setAlgaeIntakeMotorsRotation(double speed, boolean isIntake){
    if(!(isIntake)) speed *= -1; 
    setAlgaeIntakeMotorsSpeeds(speed);
  }

}
