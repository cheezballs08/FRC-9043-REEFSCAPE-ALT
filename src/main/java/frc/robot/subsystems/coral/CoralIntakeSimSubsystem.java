// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.coral;

import frc.robot.utils.Logger;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.constants.CoralIntakeConstants;
import frc.robot.constants.RobotConstants;

public class CoralIntakeSimSubsystem extends SubsystemBase implements CoralIntakeSubsystem {

  double intakeSpeed = 0;

  SingleJointedArmSim simulation;

  ProfiledPIDController angleController;

  MechanismLigament2d ligament;

  public CoralIntakeSimSubsystem() {
    this.simulation = new SingleJointedArmSim(
      DCMotor.getNEO(1), 
      CoralIntakeConstants.gearing, 
      CoralIntakeConstants.momentOfInertia, 
      CoralIntakeConstants.intakeLength, 
      CoralIntakeConstants.minimumAngle, 
      CoralIntakeConstants.maximumAngle, 
      true, 
      CoralIntakeConstants.startingAngle, 
      0,
      0
    );

    this.angleController = new ProfiledPIDController(
      CoralIntakeConstants.P, 
      CoralIntakeConstants.I, 
      CoralIntakeConstants.D, 
      CoralIntakeConstants.constraints
    );

    this.ligament = new MechanismLigament2d("IntakeLigament", CoralIntakeConstants.intakeLength, CoralIntakeConstants.startingAngle);
  }

  public double clampVoltage(double voltage) {
    if (voltage > RobotConstants.robotVoltage) {
      return RobotConstants.robotVoltage;
    }
    else if (voltage < 0) {
      return 0;
    }

    return voltage;
  }


  @Override
  public void periodic() {
    // TODO: Bunlar niye değişmiyor
    Logger.log("CoralIntake/Speeds/IntakeMotor", intakeSpeed);
    Logger.log("CoralIntake/Speeds/AngleMotor", simulation.getVelocityRadPerSec());

    Logger.log("CoralIntake/Voltages/IntakeMotor", intakeSpeed * RobotConstants.robotVoltage);
    Logger.log("CoralIntake/Voltages/AngleMotor", simulation.getVelocityRadPerSec() * RobotConstants.robotVoltage);
    
    Logger.log("CoralIntake/Encoder/Position", simulation.getAngleRads());
    Logger.log("CoralIntake/Encoder/Velocity", simulation.getVelocityRadPerSec());
    
    // Bunlarda problem yok
    Logger.log("CoralIntake/Controller/SetpointPosition", angleController.getSetpoint().position);
    Logger.log("CoralIntake/Controller/SetpointVelocity", angleController.getSetpoint().velocity);
    Logger.log("CoralIntake/Controller/PositionError", angleController.getPositionError());
    Logger.log("CoralIntake/Controller/VelocityError", angleController.getVelocityError());
    Logger.log("CoralIntake/Controller/AccumulatedError", angleController.getAccumulatedError());
    Logger.log("CoralIntake/Controller/AtSetpoint", angleController.atSetpoint());

    ligament.setAngle(Units.radiansToDegrees(simulation.getAngleRads()));
  }

  @Override
  public void setSpeeds(double speed) {
    intakeSpeed = speed;
  }

  @Override
  public void setAngle(double angle) {
    double speed = angleController.calculate(Units.radiansToDegrees(simulation.getAngleRads()), angle);

    double output = this.clampVoltage(speed * 12);

    simulation.setInputVoltage(output);
  }

  @Override
  public boolean isAtSetpoint() {
    // TODO: nedese bu doğru düzgün çalışmıyor, buna bir çözüm bul.
    return angleController.atGoal();
  }

  @Override
  public boolean isSensorActive() {
    return true;
  }

  @Override
  public Trigger getSensorAsTrigger() {
    return new Trigger(this::isSensorActive);
  }

  public MechanismLigament2d getLigament() {
    return ligament;
  }
}
