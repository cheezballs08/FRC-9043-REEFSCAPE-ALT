package frc.robot.constants;

import com.pathplanner.lib.config.RobotConfig;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;

public class RobotConstants {
  
  public static final Pose2d initialPose = new Pose2d();

  public static final double robotMass = 0.0;
  public static final double robotMomentOfInertia = 0.0;

  public static final RobotConfig config = new RobotConfig(
    robotMass, 
    robotMomentOfInertia, 
    ModuleConstants.moduleConfig, 
    new Translation2d(DrivetrainConstants.moduleLateralDistance / 2, DrivetrainConstants.moduleLongitudinalDistance / 2), // Front Left
    new Translation2d(-DrivetrainConstants.moduleLateralDistance / 2, DrivetrainConstants.moduleLongitudinalDistance / 2), // Front Right
    new Translation2d(DrivetrainConstants.moduleLateralDistance / 2, -DrivetrainConstants.moduleLongitudinalDistance / 2), // Back Left
    new Translation2d(-DrivetrainConstants.moduleLateralDistance / 2, -DrivetrainConstants.moduleLongitudinalDistance / 2) // Back Right
  );
}
