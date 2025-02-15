// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;
import org.photonvision.simulation.VisionSystemSim;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.units.VisionProcessingUnit;
import frc.robot.utils.CameraPosition;

public class Robot extends TimedRobot {
  private Command autonomousCommand;

  private final RobotContainer robotContainer;

  private VisionProcessingUnit frontUnit = VisionProcessingUnit.getUnit(CameraPosition.Front);
  private VisionProcessingUnit leftUnit = VisionProcessingUnit.getUnit(CameraPosition.Left);
  private VisionProcessingUnit rightUnit = VisionProcessingUnit.getUnit(CameraPosition.Right);

  private VisionSystemSim visionSimulation = VisionProcessingUnit.getSimulation();

  public Robot() {
    robotContainer = new RobotContainer();
  }

  public void updateVisionUnits() {
    frontUnit.update();
    leftUnit.update();
    rightUnit.update();
  }

  @Override
  public void robotPeriodic() {
    this.updateVisionUnits();
    this.robotContainer.mechansimSim.update();
  
    CommandScheduler.getInstance().run();
  }

  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  @Override
  public void disabledExit() {}

  @Override
  public void autonomousInit() {
    frontUnit.disableVisionEstimation(true);
    leftUnit.disableVisionEstimation(true);
    rightUnit.disableVisionEstimation(true);

    autonomousCommand = robotContainer.getAutonomousCommand();

    if (autonomousCommand != null) {
      autonomousCommand.schedule();
    }
  }

  @Override
  public void autonomousPeriodic() {}

  @Override
  public void autonomousExit() {
    frontUnit.disableVisionEstimation(false);
    leftUnit.disableVisionEstimation(false);
    rightUnit.disableVisionEstimation(false);
  }

  @Override
  public void teleopInit() {
    frontUnit.disableVisionEstimation(false);
    leftUnit.disableVisionEstimation(false);
    rightUnit.disableVisionEstimation(false);
 
    if (autonomousCommand != null) {
      autonomousCommand.cancel();
    }
  }

  @Override
  public void teleopPeriodic() {}

  @Override
  public void teleopExit() {}

  @Override
  public void simulationInit() {
    visionSimulation.update(robotContainer.drivetrainSubsystem.getSimPose());
  }

  @Override
  public void simulationPeriodic() {
    visionSimulation.update(robotContainer.drivetrainSubsystem.getSimPose());
  }

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void testPeriodic() {}

  @Override
  public void testExit() {}
}
