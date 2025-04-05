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

public class MiddleEscapeRightPlan extends ParallelCommandGroup {

  public MiddleEscapeRightPlan(
      DrivetrainSubsystem drivetrain,
      ElevatorSubsystem elevatorsub,
      CoralIntakeSubsystem intakesub) {
    AutoCreationCmd autodrive = new AutoCreationCmd();
    ElevatorSubsystem elevator = elevatorsub;
    CoralIntakeSubsystem intake = intakesub;
    // Auto Driving Commands

    Command centerDrive = // goes from the middle to the middle
        autodrive.AutoDriveCmd(
            drivetrain,
            List.of(new Translation2d(0.5, 0.0)),
            new Pose2d(1.53, 0, new Rotation2d(0)));

    Command escapeout = // goes from the middle to the middle
            autodrive.AutoDriveCmd(
                drivetrain,
                List.of(new Translation2d(0, 0.5)),
                new Pose2d(0, 2.7, new Rotation2d(Math.PI/2)));

    Command escapeintake = // goes from the middle to the middle
                autodrive.AutoDriveCmd(
                    drivetrain,
                    List.of(new Translation2d(0, -0.5)),
                    new Pose2d(0, -3, new Rotation2d(Math.PI/2)));

    // Driving groups
    addCommands(
        centerDrive
            .andThen(new ElevAPIDCmd(elevator, ElevatorConstants.kElevL4Ht-1))
            .andThen(new AutoSleepCmd(.5))
            .andThen(new AutoCoralSpinReverseCmd(intake, 1))
            .andThen(new ElevAPIDCmd(elevator, ElevatorConstants.kElevLowHt))
            .andThen(escapeout)
            .andThen(escapeintake));
  }
}
