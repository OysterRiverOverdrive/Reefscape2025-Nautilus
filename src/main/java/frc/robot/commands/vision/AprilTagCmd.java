// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.vision;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.auto.AutoCreationCmd;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.VisionSubsystem;
import java.util.List;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class AprilTagCmd extends Command {
  private final VisionSubsystem vision;
  private final DrivetrainSubsystem drive;
  private final AutoCreationCmd autodrive;
  Pose2d tagPose = new Pose2d(13.7244729, 2.87331191, new Rotation2d(2 * Math.PI / 3));
  boolean kIsFinished;

  /** Creates a new AprilTagCmd. */
  public AprilTagCmd(VisionSubsystem visionsub, DrivetrainSubsystem drivetrain) {
    vision = visionsub;
    drive = drivetrain;
    autodrive = new AutoCreationCmd();
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(vision);
    addRequirements(drive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    autodrive.AutoDriveCmd(
        drive,
        List.of(tagPose.minus(vision.estConsumer.getPose2d()).div(2).getTranslation()),
        (new Pose2d()).plus(tagPose.minus(vision.estConsumer.getPose2d())));
    kIsFinished =
        autodrive.AutoDriveCmd(
                drive,
                List.of(tagPose.minus(vision.estConsumer.getPose2d()).div(2).getTranslation()),
                (new Pose2d()).plus(tagPose.minus(vision.estConsumer.getPose2d())))
            .isFinished();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (kIsFinished != true) {
      return false;
    }
    return true;
  }
}
