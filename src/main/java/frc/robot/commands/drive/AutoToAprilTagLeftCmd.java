// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drive;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.auto.AutoCreationCmd;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.LimelightSubsystem;
import java.util.List;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AutoToAprilTagLeftCmd extends ParallelCommandGroup {

  boolean cmdFinished = false;
  double speed = 0.0;
  final double speedKp = -0.01;
  final double turnRateKp = 0.015;

  public AutoToAprilTagLeftCmd(LimelightSubsystem limelightsub, DrivetrainSubsystem drivetrain) {
    AutoCreationCmd autodrive = new AutoCreationCmd();
    LimelightSubsystem limelight = limelightsub;
    DrivetrainSubsystem drive = drivetrain;
    if (limelight.isAprilTag("")) {
      addCommands(
          autodrive.AutoDriveCmd(
              drive,
              List.of(limelight.ScoreLeftMotion().div(2).getTranslation()),
              limelight.ScoreLeftMotion()));
    } else {
      System.out.println("Nothing executed.");
    }
  }
}
