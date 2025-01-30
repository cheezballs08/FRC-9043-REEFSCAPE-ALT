package frc.robot.constants;

import org.photonvision.PhotonPoseEstimator.PoseStrategy;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Transform3d;

public class VisionConstants {
  public static final String cameraName = "cam0";

  public static final Transform3d robotToCamTransform = new Transform3d();

  public static final PoseStrategy poseEstimationStrategy = PoseStrategy.LOWEST_AMBIGUITY;

  // TODO: Bunu güncel değerine çevir".
  public static final AprilTagFieldLayout fieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2024Crescendo);   
}
