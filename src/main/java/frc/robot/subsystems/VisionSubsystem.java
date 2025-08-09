// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonTrackedTarget;
import frc.robot.Constants;
import frc.robot.Constants.Vision;

public class VisionSubsystem extends SubsystemBase {

    public Pose3d getPoseEstimate() {
        for (int i = 0; i < length(cameras); i++) {
            PhotonCamera cam = cameras[i];
            var result = cam.getLatestResult();
            if (result.hasTargets()) {
                PhotonTrackedTarget target = result.getBestTarget();
                int targetID = target.getFiducialId();
                
            }
        }
    }



  public void periodic() {
    // Query the latest result from PhotonVision
    var result = camera.getLatestResult();

    if (result.hasTargets()) {
      // Get the current best target.
        PhotonTrackedTarget target = result.getBestTarget();
        int targetID = target.getFiducialId();

        SmartDashboard.putNumber("Fiducial ID", targetID);
        }
    }
}
