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

public class LeftThreeCoralPlan extends ParallelCommandGroup {

  public LeftThreeCoralPlan(DrivetrainSubsystem drivetrain) {
    AutoCreationCmd autodrive = new AutoCreationCmd();

    // Auto Driving Commands

    Command showyDrive1 = // goes from left side to reef, then turns to face the reef
        autodrive.AutoDriveCmd(
            drivetrain,
            List.of(new Translation2d(2, 0.5)),
            new Pose2d(3.52, 1.01, new Rotation2d(Math.PI * 2 / 3)));
    // Place coral and get algae
    Command showyDrive2 = // goes backwards to get coral, turns to face the coral getting place
        autodrive.AutoDriveCmd(
            drivetrain,
            List.of(new Translation2d(-2.3, -0.5)),
            new Pose2d(-2.98, -1.04, new Rotation2d(Math.PI)));
    // Get coral
    Command showyDrive3 = // goes back to the reef and turns to face the reef again
        autodrive.AutoDriveCmd(
            drivetrain,
            List.of(new Translation2d(-0.68, -0.5)),
            new Pose2d(-1.77, -1.09, new Rotation2d(Math.PI)));
    // after going back, place coral

    // Driving groups

    showyDrive1
        .andThen(new AutoSleepCmd(0.25))
        .andThen(showyDrive2)
        .andThen(new AutoSleepCmd(0.25))
        .andThen(showyDrive3)
        .andThen(new AutoSleepCmd(0.25))
        .andThen(showyDrive2)
        .andThen(new AutoSleepCmd(0.25))
        .andThen(showyDrive3);
  }
}
