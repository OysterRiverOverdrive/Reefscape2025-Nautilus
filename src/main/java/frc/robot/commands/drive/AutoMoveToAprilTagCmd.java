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
  /** Creates a new AutoMoveToAprilTagCmd. */
  public AutoMoveToAprilTagCmd(
    LimelightSubsystem limelightsub, 
    DrivetrainSubsystem drivetrain) {
  AutoCreationCmd autodrive = new AutoCreationCmd();
  LimelightSubsystem limelight = limelightsub;

  //Auto Driving Commands
  Command toReef =
      autodrive.AutoDriveCmd(
        drivetrain,
        List.of(new Translation2d(sjdilfjasld)),
        new Pose2d(asdl;kjfa));
  //


    addCommands();
  }
}
