// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.utils.LimelightHelpers;
import frc.robot.Constants.LimelightConstants;
import java.util.Optional;

public class LimelightSubsystem extends SubsystemBase {
  private final SendableChooser<String> led_chooser = new SendableChooser<>();
  private final String leds_on = "leds_on";
  private final String leds_off = "leds_off";
  private final String leds_flash = "leds_flash";

  DrivetrainSubsystem drivetrain;

  AprilTagFieldLayout fieldmap;

  /** Creates a new LimelightSubSys. */
  public LimelightSubsystem(DrivetrainSubsystem drivetrain) {

    this.drivetrain = drivetrain;

    // Default to LEDs off.
    led_chooser.setDefaultOption("Off", leds_off);
    led_chooser.addOption("On", leds_on);
    led_chooser.addOption("Flash", leds_flash);
    SmartDashboard.putData("Limelight LEDs", led_chooser);

    // makes camera poses returned relative to the robots pose
    LimelightHelpers.setCameraPose_RobotSpace(
        "",
        LimelightConstants.CameraForwardOffset,
        LimelightConstants.CameraSideOffset,
        LimelightConstants.CameraUpOffest,
        LimelightConstants.CameraRollOffset,
        LimelightConstants.CameraPitchOffset,
        LimelightConstants.CameraYawOffset);

    fieldmap.loadField(AprilTagFields.k2025Reefscape);
  }

  // PoseEstimator PEstimator = new PoseEstimator<>(null, null, null, null); Pose Estimator, idk.

  public void setLEDsOn() {
    LimelightHelpers.setLEDMode_ForceOn("");
  }

  public void setLEDsOff() {
    LimelightHelpers.setLEDMode_ForceOff("");
  }

  /**
   * Gets the alliance-relative 2D pose computed from AprilTags
   *
   * @return Pose2d
   */
  public Pose2d getPose2dMegaTag1() {
    if (DriverStation.getAlliance().equals(Optional.of(Alliance.Blue))) {
      return LimelightHelpers.getBotPose2d_wpiBlue("");
    } else {
      return LimelightHelpers.getBotPose2d_wpiRed("");
    }
  }

  public LimelightHelpers.PoseEstimate getPose2dMegaTag2() {
    if (DriverStation.getAlliance().equals(Optional.of(Alliance.Blue))) {
      return LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2("");
    } else {
      return LimelightHelpers.getBotPoseEstimate_wpiRed_MegaTag2("");
    }
  }

  /**
   * Gets the alliance-relative 3D pose computed from AprilTags
   *
   * @return Pose3d
   */
  public Pose3d getPose3d() {
    if (DriverStation.getAlliance().equals(Optional.of(Alliance.Blue))) {
      return LimelightHelpers.getBotPose3d_wpiBlue("");
    } else {
      return LimelightHelpers.getBotPose3d_wpiRed("");
    }
  }

  /**
   * @return ID of primary AprilTag (if any) currently in view of the Limelight.
   */
  public int getAprilTagID() {
    return (int) LimelightHelpers.getFiducialID("");
  }

  /**
   * @return horizontal offset from crosshair to current AprilTag in degrees
   */
  public double getAprilTagX() {
    return LimelightHelpers.getTX("");
  }

  /**
   * @return vertical offset from crosshair to current AprilTag in degrees
   */
  public double getAprilTagY() {
    return LimelightHelpers.getTY("");
  }

  /**
   * @return area of the image containing AprilTag (0 to 100%)
   */
  public double getAprilTagArea() {
    return LimelightHelpers.getTA("");
  }

  /**
   * @param limelight name of the limelight (default name is "")
   * @return if there a april tag visible to the limelight
   */
  public boolean isAprilTag(String limelight) {
    return LimelightHelpers.getTV(limelight);
  }

  public double FieldApriltagX(int ID) {
    return fieldmap.getTagPose(ID).get().getX();
  }

  public double FieldApriltagY(int ID) {
    return fieldmap.getTagPose(ID).get().getY();
  }

  @Override
  public void periodic() {
    // Turn camera LEDs off or on
    if (led_chooser.getSelected().equals(leds_off)) {
      LimelightHelpers.setLEDMode_ForceOff("");
    } else if (led_chooser.getSelected().equals(leds_on)) {
      LimelightHelpers.setLEDMode_ForceOn("");
    } else {
      LimelightHelpers.setLEDMode_PipelineControl("");
    }
    SmartDashboard.putNumber("Current Apriltag ID", LimelightHelpers.getFiducialID(""));
    SmartDashboard.putNumber("X Distance", getPose2dMegaTag2().pose.getX());
    SmartDashboard.putNumber("Y Distance", getPose2dMegaTag2().pose.getY());
    SmartDashboard.putNumber(
        "Rotation in Rads", getPose2dMegaTag2().pose.getRotation().getRadians());

    if (isAprilTag("")) {
      Pose2d curP = getPose2dMegaTag2().pose;
      double xDist = curP.getX();
      double yDist = curP.getY();
      Rotation2d origAngle = curP.getRotation();

      double curTag = LimelightHelpers.getFiducialID("");

      // placeholder 6 apriltag ~position
      double tagX = 113;
      double tagY = 162;

      double moveX = tagX - xDist;
      double moveY = tagY - yDist;
      double mag = Math.hypot(tagX, tagY);

      Rotation2d rot = new Rotation2d(Math.atan2(moveY, moveX));

      double finalXD = mag * Math.cos(rot.minus(origAngle).getRadians());
      double finalYD = mag * Math.sin(rot.minus(origAngle).getRadians());
      Pose2d finalPose = new Pose2d(finalXD, finalYD, new Rotation2d(origAngle.getRadians()));

      // Auto Driving Commands

      SmartDashboard.putNumber("finalPoseX", finalXD);
      SmartDashboard.putNumber("finalPoseY", finalYD);
      SmartDashboard.putNumber("newPoseRotation", rot.minus(origAngle).getRadians());
    }
  }
}
