package frc.robot.auto.plans;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.auto.AutoCreationCmd;
import frc.robot.subsystems.DrivetrainSubsystem;
import java.util.List;

// STARTS ON THE RIGHT SIDE

public class MiddleOneCoralPlan extends ParallelCommandGroup {

  public MiddleOneCoralPlan(DrivetrainSubsystem drivetrain) {
    AutoCreationCmd autodrive = new AutoCreationCmd();

    // Auto Driving Commands

    Command showyDrive1 = // goes from the middle to the middle
        autodrive.AutoDriveCmd(
            drivetrain,
            List.of(new Translation2d(1, 0.01)),
            new Pose2d(1.21, .02, new Rotation2d(0)));
    // Place coral

    // Driving groups
    addCommands(showyDrive1);
  }
}
