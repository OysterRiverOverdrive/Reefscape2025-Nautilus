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

public class LeftOneCoralPlan extends ParallelCommandGroup {

  public LeftOneCoralPlan(DrivetrainSubsystem drivetrain) {
    AutoCreationCmd autodrive = new AutoCreationCmd();

    // Auto Driving Commands

    Command showyDrive1 = // goes from left side to reef, then turns to face the reef
        autodrive.AutoDriveCmd(
            drivetrain,
            List.of(new Translation2d(1, 0.5)),
            new Pose2d(1.76, 1.01, new Rotation2d(Math.PI * 1 / 3)));
    // Place coral

    // Driving groups

    andThen(showyDrive1);
  }
}
