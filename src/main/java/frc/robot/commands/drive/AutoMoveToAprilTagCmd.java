// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drive;

import static edu.wpi.first.units.Units.Meters;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.auto.AutoCreationCmd;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.LimelightSubsystem;
import frc.utils.LimelightHelpers;
import java.util.List;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AutoMoveToAprilTagCmd extends ParallelCommandGroup {

  boolean cmdFinished = false;
  double speed = 0.0;
  final double speedKp = -0.01;
  final double turnRateKp = 0.015;

  public AutoMoveToAprilTagCmd(LimelightSubsystem limelightsub, DrivetrainSubsystem drivetrain) {
    AutoCreationCmd autodrive = new AutoCreationCmd();
    LimelightSubsystem limelight = limelightsub;
    DrivetrainSubsystem drive = drivetrain;
    if (limelight.isAprilTag("")) {
      Pose2d curP = limelight.getPose2dMegaTag2().pose;
      double xDist = curP.getX();
      double yDist = curP.getY();
      Rotation2d origAngle = curP.getRotation();

      int curTag = (int) LimelightHelpers.getFiducialID("");

      // placeholder 6 apriltag ~position
      // double tagX = 113;
      // double tagY = 162;

      Pose2d tagPose = limelight.FieldApriltagPose(curTag);

      double moveX = tagPose.getMeasureX().minus(curP.getMeasureX()).in(Meters);
      double moveY = tagPose.getMeasureY().minus(curP.getMeasureY()).in(Meters);
      double mag = Math.hypot(moveX, moveY);

      Rotation2d rot = new Rotation2d(moveX, moveY);

      double finalXD = mag * Math.cos(rot.minus(origAngle).getRadians());
      double finalYD = mag * Math.sin(rot.minus(origAngle).getRadians());
      Pose2d finalPose =
          new Pose2d(finalXD, finalYD, new Rotation2d(rot.minus(origAngle).getRadians()));

      // Auto Driving Commands
      Command toAprilTag =
          autodrive.AutoDriveCmd(drive, List.of(finalPose.div(2).getTranslation()), finalPose);
      //
      SmartDashboard.putNumber("finalPoseX", finalXD);
      SmartDashboard.putNumber("finalPoseY", finalYD);
      SmartDashboard.putNumber("newPoseRotation", rot.minus(origAngle).getRadians());
    } else {
      System.out.println("Nothing executed.");
    }
    addCommands();
  }
}
