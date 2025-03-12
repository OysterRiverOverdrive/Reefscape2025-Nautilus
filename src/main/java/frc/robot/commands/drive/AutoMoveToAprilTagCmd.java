// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drive;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.auto.AutoCreationCmd;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.LimelightSubsystem;
import java.util.List;

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
    DrivetrainSubsystem drivetrain) 
    {
    AutoCreationCmd autodrive = new AutoCreationCmd();
    LimelightSubsystem limelight = limelightsub;

  //Auto Driving Commands
  Command toReef =
      autodrive.AutoDriveCmd(
        drivetrain,
        List.of(new Translation2d(0,0)),
        new Pose2d(0,0,new Rotation2d(0)));
  //
    addCommands();
  }
}
