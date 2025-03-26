// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Inches;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.LimelightConstants;
import frc.utils.LimelightHelpers;
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
        LimelightConstants.kCameraForwardOffset.in(Inches),
        LimelightConstants.kCameraSideOffset.in(Inches),
        LimelightConstants.kCameraUpOffest.in(Inches),
        LimelightConstants.kCameraRollOffset.in(Inches),
        LimelightConstants.kCameraPitchOffset.in(Inches),
        LimelightConstants.kCameraYawOffset.in(Inches));
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

  /**
   * Returns an April Tag Pose, rotated so the Rotation2D faces toward the april tag. The native
   * Rotation2D faces outward from the April Tag, so the native pose is deconstructed, and a rotated
   * pose is constructed with the same Translation (x and y), and the Rotation is the native
   * rotation plus PI to fully flip the direction.
   *
   * @param ID april tage ID
   * @return tag pose relative to origin
   */
  public Pose2d FieldApriltagPose(int ID) {
    Pose2d tagPose = fieldmap.getTagPose(ID).get().toPose2d();
    Pose2d tagPoseRotated =
        new Pose2d(tagPose.getTranslation(), tagPose.getRotation().plus(new Rotation2d(Math.PI)));
    return tagPoseRotated;
  }

  public double FieldApriltagX(int ID) {
    return FieldApriltagPose(ID).getX();
  }

  public double FieldApriltagY(int ID) {
    return FieldApriltagPose(ID).getY();
  }

  public Pose2d AprilTagOutsetPose() {
    int curTag = (int) LimelightHelpers.getFiducialID("");

    Pose2d tagPose = FieldApriltagPose(curTag);

    // tagPose, offset so as to put the center of the robot outside of the bounds of the reef
    // This distance is 16.75 inches
    Transform2d outset =
        new Transform2d(
            Distance.ofRelativeUnits(-16.75, Inches),
            Distance.ofRelativeUnits(0, Inches),
            new Rotation2d(0));
    Pose2d tagPoseOutset = tagPose.transformBy(outset);
    return tagPoseOutset;
  }

  // For Testing
  public Pose2d AprilTagCenterMotion() {
    Pose2d curP = getPose2dMegaTag2().pose;
    Transform2d motion = AprilTagOutsetPose().minus(curP);
    Pose2d motionPose = new Pose2d(motion.getTranslation(), motion.getRotation());
    return motionPose;
  }

  // Since scoring may happen on either the left or the right, there must be two
  // different locations, leftPose and rightPose
  public Pose2d ScoreLeftPose() {
    Transform2d leftOffset =
        new Transform2d(
            Distance.ofRelativeUnits(0, Inches),
            LimelightConstants.kCoralPostOffset.minus(LimelightConstants.kIntakeOffset),
            new Rotation2d(0));

    Pose2d leftPose = AprilTagOutsetPose().transformBy(leftOffset);
    return leftPose;
  }

  public Pose2d ScoreRightPose() {
    Transform2d rightOffset =
        new Transform2d(
            Distance.ofRelativeUnits(0, Inches),
            LimelightConstants.kCoralPostOffset
                .unaryMinus()
                .minus(LimelightConstants.kIntakeOffset),
            new Rotation2d(0));

    Pose2d rightPose = AprilTagOutsetPose().transformBy(rightOffset);
    return rightPose;
  }

  // Need different motions for different end positions
  public Pose2d ScoreLeftMotion() {
    Pose2d curP = getPose2dMegaTag2().pose;
    Transform2d leftMotion = ScoreLeftPose().minus(curP);
    Pose2d leftMotionPose = new Pose2d(leftMotion.getTranslation(), leftMotion.getRotation());
    return leftMotionPose;
  }

  public Pose2d ScoreRightMotion() {
    Pose2d curP = getPose2dMegaTag2().pose;
    Transform2d rightMotion = ScoreRightPose().minus(curP);
    Pose2d rightMotionPose = new Pose2d(rightMotion.getTranslation(), rightMotion.getRotation());
    return rightMotionPose;
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
    SmartDashboard.putBoolean("Is April Tag", isAprilTag(""));
    if (isAprilTag("")) {
      SmartDashboard.putNumber("Current Apriltag ID", LimelightHelpers.getFiducialID(""));
    }
  }
}
