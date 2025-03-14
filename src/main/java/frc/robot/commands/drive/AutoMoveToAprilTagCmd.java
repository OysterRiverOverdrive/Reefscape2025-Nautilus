// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drive;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.auto.AutoCreationCmd;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.LimelightSubsystem;
import frc.utils.LimelightHelpers;
import java.util.List;

import org.opencv.core.Mat;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AutoMoveToAprilTagCmd extends ParallelCommandGroup {
  DrivetrainSubsystem drv;
  LimelightSubsystem camera;

  boolean cmdFinished = false;
  double speed = 0.0;
  final double speedKp = -0.01;
  final double turnRateKp = 0.015;

  public AutoMoveToAprilTagCmd(LimelightSubsystem limelightsub, DrivetrainSubsystem drivetrain) {
    AutoCreationCmd autodrive = new AutoCreationCmd();
    LimelightSubsystem limelight = limelightsub;
    if (limelight.isAprilTag("")) {
      Pose2d curP = limelight.getPose2dMegaTag2().pose;
      double xDist = curP.getX();
      double yDist = curP.getY();
      Rotation2d origAngle = curP.getRotation();

      double curTag = LimelightHelpers.getFiducialID("");

      // placeholder 6 apriltag ~position

      double moveX = limelight.FieldApriltagX((int)curTag) - xDist;
      double moveY = limelight.FieldApriltagY((int)curTag) - yDist;
      double mag = Math.sqrt(Math.pow(limelight.FieldApriltagX((int)curTag), 2) + Math.pow(limelight.FieldApriltagY((int)curTag),2));

      Rotation2d rot = new Rotation2d(Math.atan2(moveY, moveX));

      double finalXD = mag * Math.cos(rot.minus(origAngle).getRadians());
      double finalYD = mag * Math.sin(rot.minus(origAngle).getRadians());
      Pose2d finalPose = new Pose2d(finalXD, finalYD, new Rotation2d(60 - origAngle.getRadians()));

      // Auto Driving Commands
      Command toAprilTag =
          autodrive.AutoDriveCmd(drivetrain, List.of(finalPose.div(2).getTranslation()), finalPose);
      //
      addCommands(toAprilTag);
    } else {
      addCommands(null);
    }
  }
}
