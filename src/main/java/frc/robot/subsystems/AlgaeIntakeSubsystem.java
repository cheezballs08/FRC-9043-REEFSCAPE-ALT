// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class AlgaeIntakeSubsystem extends SubsystemBase {
    SparkMax algaeIntakeMotor1, algaeIntakeMotor2;

    //Constants
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

  public void SetAlgaeIntakeMotorsSpeeds(double speed){
    algaeIntakeMotor1.set(speed);
    algaeIntakeMotor2.set(speed);
  }

  public void SetAlgaeIntakeMotorsRotation(double speed, boolean isIntake){
    if(!(isIntake)) speed *= -1; 
    SetAlgaeIntakeMotorsSpeeds(speed);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
