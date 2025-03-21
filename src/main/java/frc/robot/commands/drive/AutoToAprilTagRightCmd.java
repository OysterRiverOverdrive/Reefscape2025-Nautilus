// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drive;

import static edu.wpi.first.units.Units.Inches;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.Constants.LimelightConstants;
import frc.robot.auto.AutoCreationCmd;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.LimelightSubsystem;
import frc.utils.LimelightHelpers;
import java.util.List;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AutoToAprilTagRightCmd extends ParallelCommandGroup {

  boolean cmdFinished = false;
  double speed = 0.0;
  final double speedKp = -0.01;
  final double turnRateKp = 0.015;

  public AutoToAprilTagRightCmd(LimelightSubsystem limelightsub, DrivetrainSubsystem drivetrain) {
    AutoCreationCmd autodrive = new AutoCreationCmd();
    LimelightSubsystem limelight = limelightsub;
    DrivetrainSubsystem drive = drivetrain;
    if (limelight.isAprilTag("")) {
      Pose2d curP = limelight.getPose2dMegaTag2().pose;

      int curTag = (int) LimelightHelpers.getFiducialID("");

      Pose2d tagPose = limelight.FieldApriltagPose(curTag);

      // tagPose, offset so as to put the center of the robot outside of the bounds of the reef
      // This distance is 16.75 inches
      Transform2d outset =
          new Transform2d(
              Distance.ofRelativeUnits(-16.75, Inches),
              Distance.ofRelativeUnits(0, Inches),
              new Rotation2d(0));
      Pose2d tagPoseOutset = tagPose.transformBy(outset);

      Transform2d rightOffset =
          new Transform2d(
              Distance.ofRelativeUnits(0, Inches),
              LimelightConstants.kCoralPostOffset
                  .unaryMinus()
                  .minus(LimelightConstants.kIntakeOffset),
              new Rotation2d(0));

      Pose2d rightPose = tagPoseOutset.transformBy(rightOffset);

      Transform2d rightMotion = rightPose.minus(curP);

      Pose2d rightMotionPose = new Pose2d(rightMotion.getTranslation(), rightMotion.getRotation());

      Command toAprilTagRight =
          autodrive.AutoDriveCmd(
              drive, List.of(rightMotionPose.div(2).getTranslation()), rightMotionPose);

      SmartDashboard.putNumber("RightFinX", rightMotion.getX());
      SmartDashboard.putNumber("RightFinY", rightMotion.getY());
      SmartDashboard.putNumber("RightFinRot", rightMotion.getRotation().getRadians());
    } else {
      System.out.println("Nothing executed.");
    }
    addCommands();
  }
}
