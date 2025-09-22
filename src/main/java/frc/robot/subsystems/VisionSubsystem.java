// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Vision;
import frc.robot.Robot;
import java.util.ArrayList;
import java.util.List;
import java.util.Optional;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;
import org.photonvision.targeting.PhotonTrackedTarget;

public class VisionSubsystem extends SubsystemBase {
  PhotonCamera camera1;
  PhotonCamera camera2;
  PhotonCamera camera3;
  PhotonCamera camera4;

  ArrayList<PhotonCamera> cameras = new ArrayList<>();

  PhotonPoseEstimator photonEstimatorCam1;
  PhotonPoseEstimator photonEstimatorCam2;
  PhotonPoseEstimator photonEstimatorCam3;
  PhotonPoseEstimator photonEstimatorCam4;

  ArrayList<PhotonPoseEstimator> photonEstimators = new ArrayList<>();

  private Matrix<N3, N1> curStdDevs;
  public final EstimateConsumer estConsumer;

  // Simulation
  private PhotonCameraSim cameraSim;
  private VisionSystemSim visionSim;

  AprilTagFieldLayout fieldmap;

  public VisionSubsystem(EstimateConsumer estConsumer) {
    this.estConsumer = estConsumer;
    camera1 = new PhotonCamera("Camera1");
    camera2 = new PhotonCamera("Camera2");

    // Add other cameras once they are added to robot
    cameras.add(camera1);
    cameras.add(camera2);

    fieldmap = AprilTagFieldLayout.loadField(AprilTagFields.k2025Reefscape);

    photonEstimatorCam1 =
        new PhotonPoseEstimator(
            fieldmap, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, Vision.kRobotToCam1);
    photonEstimatorCam1.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);

    photonEstimatorCam2 =
        new PhotonPoseEstimator(
            fieldmap, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, Vision.kRobotToCam2);
    photonEstimatorCam2.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);

    photonEstimatorCam3 =
        new PhotonPoseEstimator(
            fieldmap, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, Vision.kRobotToCam3);
    photonEstimatorCam3.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);

    photonEstimatorCam4 =
        new PhotonPoseEstimator(
            fieldmap, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, Vision.kRobotToCam4);
    photonEstimatorCam4.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);

    // Add other photonEstimators when other cameras are added
    photonEstimators.add(photonEstimatorCam1);
    photonEstimators.add(photonEstimatorCam2);

    // ----- Simulation
    if (Robot.isSimulation()) {
      // Create the vision system simulation which handles cameras and targets on the field.
      visionSim = new VisionSystemSim("main");
      // Add all the AprilTags inside the tag layout as visible targets to this simulated field.
      visionSim.addAprilTags(fieldmap);
      // Create simulated camera properties. These can be set to mimic your actual camera.
      var cameraProp = new SimCameraProperties();
      cameraProp.setCalibration(960, 720, Rotation2d.fromDegrees(90));
      cameraProp.setCalibError(0.35, 0.10);
      cameraProp.setFPS(15);
      cameraProp.setAvgLatencyMs(50);
      cameraProp.setLatencyStdDevMs(15);
      // Create a PhotonCameraSim which will update the linked PhotonCamera's values with visible
      // targets.
      cameraSim = new PhotonCameraSim(camera1, cameraProp);
      // Add the simulated camera to view the targets on this simulated field.
      visionSim.addCamera(cameraSim, Vision.kRobotToCam1);

      cameraSim.enableDrawWireframe(true);
    }
  }

  public void periodic() {
    Optional<EstimatedRobotPose> visionEst = Optional.empty();
    for (int i = 0; i < 2; i++) {
      PhotonCamera camera = cameras.get(i);
      for (var change : camera.getAllUnreadResults()) {
        visionEst = photonEstimators.get(i).update(change);
        updateEstimationStdDevs(visionEst, change.getTargets());

        if (Robot.isSimulation()) {
          visionEst.ifPresentOrElse(
              est ->
                  getSimDebugField()
                      .getObject("VisionEstimation")
                      .setPose(est.estimatedPose.toPose2d()),
              () -> {
                getSimDebugField().getObject("VisionEstimation").setPoses();
              });
        }

        visionEst.ifPresent(
            est -> {
              // Change our trust in the measurement based on the tags we can see
              var estStdDevs = getEstimationStdDevs();

              estConsumer.accept(est.estimatedPose.toPose2d(), est.timestampSeconds, estStdDevs);
            });
      }
    }

    if (!(visionEst.isEmpty())) {
      SmartDashboard.putNumber("Robot X", visionEst.get().estimatedPose.getX());
      SmartDashboard.putNumber("Robot Y", visionEst.get().estimatedPose.getY());
    }
    ;
  }

  private void updateEstimationStdDevs(
      Optional<EstimatedRobotPose> estimatedPose, List<PhotonTrackedTarget> targets) {
    if (estimatedPose.isEmpty()) {
      // No pose input. Default to single-tag std devs
      curStdDevs = Vision.kSingleTagStdDevs;

    } else {
      // Pose present. Start running Heuristic
      var estStdDevs = Vision.kSingleTagStdDevs;
      int numTags = 0;
      double avgDist = 0;

      // Precalculation - see how many tags we found, and calculate an average-distance metric
      for (var tgt : targets) {
        for (int i = 0; i < 2; i++) {
          var tagPose = photonEstimators.get(i).getFieldTags().getTagPose(tgt.getFiducialId());
          if (tagPose.isEmpty()) continue;
          numTags++;
          avgDist +=
              tagPose
                  .get()
                  .toPose2d()
                  .getTranslation()
                  .getDistance(estimatedPose.get().estimatedPose.toPose2d().getTranslation());
        }
      }

      if (numTags == 0) {
        // No tags visible. Default to single-tag std devs
        curStdDevs = Vision.kSingleTagStdDevs;
      } else {
        // One or more tags visible, run the full heuristic.
        avgDist /= numTags;
        // Decrease std devs if multiple targets are visible
        if (numTags > 1) estStdDevs = Vision.kMultiTagStdDevs;
        // Increase std devs based on (average) distance
        if (numTags == 1 && avgDist > 4)
          estStdDevs = VecBuilder.fill(Double.MAX_VALUE, Double.MAX_VALUE, Double.MAX_VALUE);
        else estStdDevs = estStdDevs.times(1 + (avgDist * avgDist / 30));
        curStdDevs = estStdDevs;
      }
    }
  }

  /**
   * Returns the latest standard deviations of the estimated pose from {@link
   * #getEstimatedGlobalPose()}, for use with {@link
   * edu.wpi.first.math.estimator.SwerveDrivePoseEstimator SwerveDrivePoseEstimator}. This should
   * only be used when there are targets visible.
   */
  public Matrix<N3, N1> getEstimationStdDevs() {
    return curStdDevs;
  }

  // ----- Simulation

  public void simulationPeriodic(Pose2d robotSimPose) {
    visionSim.update(robotSimPose);
  }

  /** Reset pose history of the robot in the vision system simulation. */
  public void resetSimPose(Pose2d pose) {
    if (Robot.isSimulation()) visionSim.resetRobotPose(pose);
  }

  /** A Field2d for visualizing our robot and objects on the field. */
  public Field2d getSimDebugField() {
    if (!Robot.isSimulation()) return null;
    return visionSim.getDebugField();
  }
}
