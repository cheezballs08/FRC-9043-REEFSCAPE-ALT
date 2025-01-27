package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class AlgaeIntakeSubsystem extends SubsystemBase {
  
  SparkMax algaeIntakeMotor1, algaeIntakeMotor2;

  // Sabitler
  int algaeIntakeMotor1ID = 0, algaeIntakeMotor2ID = 0;
  
  MotorType algaeIntakeMotor1Type = null, algaeIntakeMotor2Type = null;

  SparkBaseConfig algeaMotorDefaultConfig = new SparkMaxConfig()
  .smartCurrentLimit(0)
  .idleMode(null);

  SparkBaseConfig motor1Config = new SparkMaxConfig()
  .apply(algeaMotorDefaultConfig)
  .inverted(false);

  SparkBaseConfig motor2Config = new SparkMaxConfig()
  .apply(algeaMotorDefaultConfig)
  .inverted(false);

  // TODO: Bu persist ve reset mode'u genel bir sabit sınıfına atarak her alt birimide aynı kullanılmasını sağla
  PersistMode persistMode = PersistMode.kPersistParameters;

  ResetMode resetMode = ResetMode.kNoResetSafeParameters;

  public AlgaeIntakeSubsystem() {
    this.algaeIntakeMotor1 = new SparkMax(algaeIntakeMotor1ID, algaeIntakeMotor1Type);
    this.algaeIntakeMotor1.configure(motor1Config, resetMode, persistMode);

    this.algaeIntakeMotor2 = new SparkMax(algaeIntakeMotor2ID, algaeIntakeMotor2Type);
    this.algaeIntakeMotor2.configure(motor1Config, resetMode, persistMode);
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
