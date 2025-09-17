// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.vision;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.auto.AutoCreationCmd;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.VisionSubsystem;
import java.util.List;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ToAprilTagCmd extends ParallelCommandGroup {

  boolean cmdFinished = false;
  double speed = 0.0;
  final double speedKp = -0.01;
  final double turnRateKp = 0.015;

  Pose3d tagPose = new Pose3d(13.7244729, 2.87331191, 0, new Rotation3d(0, 0, 2 * Math.PI / 3))

  // FOR TESTING ONLY!!!
  public ToAprilTagCmd(VisionSubsystem visionsub, DrivetrainSubsystem drivetrain) {
  AutoCreationCmd autodrive = new AutoCreationCmd();
  VisionSubsystem vision = visionsub;
  DrivetrainSubsystem drive = drivetrain;
    addCommands(
        autodrive.AutoDriveCmd(
            drive,
            List.of(limelight.AprilTagCenterMotion().div(2).getTranslation()),
            ));  
  }
}