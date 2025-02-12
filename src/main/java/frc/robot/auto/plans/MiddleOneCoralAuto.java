package frc.robot.auto.plans;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.auto.AutoCreationCmd;
import frc.robot.auto.AutoSleepCmd;
import frc.robot.subsystems.DrivetrainSubsystem;
import java.util.List;

// STARTS ON THE RIGHT SIDE

public class MiddleOneCoralAuto extends ParallelCommandGroup {

  public MiddleOneCoralAuto (DrivetrainSubsystem drivetrain) {
    AutoCreationCmd autodrive = new AutoCreationCmd();

    // Auto Driving Commands

    Command drive1 = // leaves zone
        autodrive.AutoDriveCmd(
            drivetrain,
            List.of(new Translation2d(0.5, 0)),
            new Pose2d(1.5, 0, new Rotation2d(0)));
    //put coral on branch

    // Driving groups

    drive1
        .andThen(new AutoSleepCmd(0.25));
  }
}
