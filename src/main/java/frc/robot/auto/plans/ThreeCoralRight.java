package frc.robot.auto.plans;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.Constants.RobotConstants.ElevatorConstants;
import frc.robot.auto.AutoCoralSpinReverseCmd;
import frc.robot.auto.AutoCreationCmd;
import frc.robot.auto.AutoSleepCmd;
import frc.robot.commands.elevator.ElevAPIDCmd;
import frc.robot.subsystems.CoralIntakeSubsystem;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import java.util.List;

// STARTS ON THE RIGHT SIDE

public class ThreeCoralRight extends ParallelCommandGroup {

  public ThreeCoralRight(
      DrivetrainSubsystem drivetrain,
      ElevatorSubsystem elevatorsub,
      CoralIntakeSubsystem intakesub) {
    AutoCreationCmd autodrive = new AutoCreationCmd();
    ElevatorSubsystem elevator = elevatorsub;
    CoralIntakeSubsystem intake = intakesub;

    // Auto Driving Commands
    Command toReef =
        autodrive.AutoDriveCmd(
            drivetrain,
            List.of(new Translation2d(3, 0.3)),
            new Pose2d(3.55, -0.06, new Rotation2d(-Math.PI * 2 / 3)));
    // Place coral and get algae
    Command toFeeder1 = // goes backwards to get coral, turns to face the coral getting place
        autodrive.AutoDriveCmd(
            drivetrain,
            List.of(new Translation2d(-2, 1)),
            new Pose2d(-3.5, 1.5, new Rotation2d(Math.PI / 2 - Math.toRadians(6))));
    // Get coral
    Command toReef1 = // goes back to the reef and turns to face the reef again (it's the same
        // thing)
        autodrive.AutoDriveCmd(
            drivetrain,
            List.of(new Translation2d(-2, 1)),
            new Pose2d(-2.7, 1.7, new Rotation2d(Math.PI)));
    // after going back, place coral

    Command toFeeder2 = // andthen doesn't like mutliple of the same command
        autodrive.AutoDriveCmd(
            drivetrain,
            List.of(new Translation2d(-2, 1)),
            new Pose2d(-2.7, 1.7, new Rotation2d(Math.PI)));
    Command toReef2 =
        autodrive.AutoDriveCmd(
            drivetrain,
            List.of(new Translation2d(-2, 1)),
            new Pose2d(-2.7, 1.7, new Rotation2d(Math.PI)));

    // Driving groups
    addCommands(
        toReef
            .andThen(new ElevAPIDCmd(elevator, ElevatorConstants.kElevL4Ht))
            .andThen(new AutoSleepCmd(0.5))
            .andThen(new AutoCoralSpinReverseCmd(intake, 1))
            .andThen(new ElevAPIDCmd(elevator, ElevatorConstants.kElevLowHt, 4))
            .andThen(new ElevAPIDCmd(elevator, ElevatorConstants.kElevIntakeHt))
            .andThen(toFeeder1));
  }
}
