// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;
package frc.robot.commands.drive;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.LimelightSubsystem;

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

  public AutoMoveToAprilTagCmd(
    LimelightSubsystem limelightsub, 
    DrivetrainSubsystem drivetrain) {
  AutoCreationCmd autodrive = new AutoCreationCmd();
  LimelightSubsystem limelight = limelightsub;

  //Auto Driving Commands
  Command toReef =
      autodrive.AutoDriveCmd(
        drivetrain,
        List.of(new Translation2d(0,0)),
        new Pose2d(,0,new Rotation2d(0)));
  //


    addCommands();
  }

  public distanceToTag() {
    if (cmdFinished == false) {
      double distToMyTgt = //CurPose.getDistance() //find command to get y distance

      if (/*can see target boolean */ && distToMyTgt < 50) // dist is inches
      {
        // speed proportions if distance less than 50 in
        speed = distToMyTgt * speedKp; // calculating speed to help make the stop smoother
      }

      if (/*can see target boolean*/ && distToMyTgt <= standOffDistInches) // the bot is at its goal
      {

        speed = 0.0;
        drv.fieldDrive(0, 0, 0, 0, 0); // stop the bot once reached the distance

        cmdFinished = true;
      }

      if (cmdFinished == false && distToMyTgt != -10000.00) {
        //drive to target
      }
    }

    return 
  }
}
