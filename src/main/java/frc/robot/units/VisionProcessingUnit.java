// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.units;

import java.util.List;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.geometry.Pose2d;
import frc.robot.Constants.VisionConstants;

public class VisionProcessingUnit {

  private static  VisionProcessingUnit instance;
  
  private PhotonCamera camera;

  private PhotonTrackedTarget bestTarget;

  private List<PhotonTrackedTarget> seenSargets;

  private PhotonPipelineResult result;

  private PhotonPoseEstimator poseEstimator;

  private EstimatedRobotPose estimatedPose;

  private VisionProcessingUnit(String cameraName) {
    camera = new PhotonCamera(cameraName);

    poseEstimator = new PhotonPoseEstimator(VisionConstants.fieldLayout, VisionConstants.poseEstimationStrategy, VisionConstants.robotToCamTransform);
  }

  public static VisionProcessingUnit getInstance() {
    if (instance == null) {
      instance = new VisionProcessingUnit(VisionConstants.cameraName);
    }
    return instance;
  }

  public void updateResult() {
    this.seenSargets.removeAll(seenSargets);

    this.result = new PhotonPipelineResult();
    
    List<PhotonPipelineResult> results = camera.getAllUnreadResults();

    if (results.isEmpty()) {
      return;
    }
    
    // TODO: Make sure this returns the latest one.
    this.result = results.get(results.size() - 1);
  }

  public void updateTargets() {
    this.seenSargets = result.getTargets();
    this.bestTarget = result.getBestTarget();
  }

  public void update() {
    this.updateResult();
    this.updateTargets();
    this.updatePoseEstimator();
  }

  public PhotonTrackedTarget getBestTarget() {
    return bestTarget;
  }

  public List<PhotonTrackedTarget> getTargets() {
    return seenSargets;
  }

  public PhotonPipelineResult getResult() {
    return result;
  }

  public boolean isSeen(int id) {
    for (PhotonTrackedTarget target : seenSargets) {
      if (target.getFiducialId() == id) {
        return true;
      }
    }
    return false;
  }

  public PhotonTrackedTarget getTarget(int id) {
    for (PhotonTrackedTarget target : seenSargets) {
      if (target.getFiducialId() == id) {
        return target;
      }
    }
    return null;
  }

  public void updatePoseEstimator() {
    // TODO: Possible termination point! Handle with more care!
    // TODO: Also, shouldn't there be more to this? Dosen't this need to be calibrated with the cameras properties?
    estimatedPose = poseEstimator.update(result).isPresent() ? poseEstimator.update(result).get() : null;  
  }

  public boolean canEstimatePose() {
    return estimatedPose != null;
  }

  public EstimatedRobotPose getEstimate() {
    return estimatedPose;
  }

  public Pose2d getEstimatedPose2d() {
    return estimatedPose.estimatedPose.toPose2d();
  }

}
