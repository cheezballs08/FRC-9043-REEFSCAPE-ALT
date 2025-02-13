// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.coral;

import frc.robot.utils.Logger;
import dev.doglog.DogLog;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose3d;
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

  ArmFeedforward feedforward;

  MechanismLigament2d ligament;

  Pose3d armPose;

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

    this.feedforward = new ArmFeedforward(
      CoralIntakeConstants.S, 
      CoralIntakeConstants.G, 
      CoralIntakeConstants.V
    );

    this.ligament = new MechanismLigament2d("IntakeLigament", CoralIntakeConstants.intakeLength, CoralIntakeConstants.startingAngle);
  
    this.armPose = new Pose3d();
  
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

  public double speedToVoltage(double speed) {
  
    if (speed > 1) {
      speed = 1;
    }
    else if (speed < -1) {
      speed = -1;
    }

    return speed * RobotConstants.robotVoltage;
  }


  @Override
  public void periodic() {
    simulation.update(0.020);

    Logger.log("CoralIntake/Speeds/IntakeMotor", intakeSpeed);

    Logger.log("CoralIntake/Encoder/Position", Units.radiansToDegrees(simulation.getAngleRads()));
    Logger.log("CoralIntake/Encoder/Velocity", Units.radiansToDegrees(simulation.getVelocityRadPerSec()));

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
    double speed = angleController.calculate(Units.radiansToDegrees(simulation.getAngleRads()), angle) + feedforward.calculate(simulation.getAngleRads(), simulation.getVelocityRadPerSec() / RobotConstants.robotVoltage);
    DogLog.log("CoralIntake/Speeds/AngleMotor", speed);

    double output = this.speedToVoltage(speed);

    simulation.setInputVoltage(output);
  }

  @Override
  public boolean isAtSetpoint() {
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

  public Pose3d getPose() {
      return null;

  }
}
