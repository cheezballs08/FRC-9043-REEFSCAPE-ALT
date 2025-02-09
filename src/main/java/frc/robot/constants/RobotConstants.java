package frc.robot.constants;

import com.pathplanner.lib.config.RobotConfig;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

public class RobotConstants {

  public static final Pose2d origin = new Pose2d();
  
  public static final Pose2d blueStart1 = new Pose2d(8.150, 7.270, Rotation2d.fromDegrees(180));
  public static final Pose2d blueStart2 = new Pose2d(8.150, 6.180, Rotation2d.fromDegrees(180));
  public static final Pose2d blueStart3 = new Pose2d(8.150, 5.085, Rotation2d.fromDegrees(180));

  public static final Pose2d redStart1 = new Pose2d();
  public static final Pose2d redStart2 = new Pose2d();
  public static final Pose2d redStart3 = new Pose2d();

  public static final Pose2d initialPose = blueStart1;

  public static final double robotMass = 74.088;
  public static final double robotMomentOfInertia = 0.883;

  public static final Alliance alliance = Alliance.Blue;

  public static RobotConfig config;
  
  static {
    try{
      config = RobotConfig.fromGUISettings();
    } catch (Exception e) {
      e.printStackTrace();
    }
  }

  /*public static final RobotConfig config = new RobotConfig(
    robotMass, 
    robotMomentOfInertia, 
    ModuleConstants.moduleConfig, 
    new Translation2d(DrivetrainConstants.moduleLateralDistance / 2, DrivetrainConstants.moduleLongitudinalDistance / 2), // Front Left
    new Translation2d(-DrivetrainConstants.moduleLateralDistance / 2, DrivetrainConstants.moduleLongitudinalDistance / 2), // Front Right
    new Translation2d(DrivetrainConstants.moduleLateralDistance / 2, -DrivetrainConstants.moduleLongitudinalDistance / 2), // Back Left
    new Translation2d(-DrivetrainConstants.moduleLateralDistance / 2, -DrivetrainConstants.moduleLongitudinalDistance / 2) // Back Right
  );*/
}
