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

public class LeftOneCoralPlan extends ParallelCommandGroup {

  public LeftOneCoralPlan(
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
            List.of(new Translation2d(3, -0.3)),
            new Pose2d(3.85, 0.29, new Rotation2d(Math.PI * 2 / 3)));
    // Place coral
    Command recenter = // goes backwards to get coral, turns to face the coral getting place
        autodrive.AutoDriveCmd(
            drivetrain,
            List.of(new Translation2d(-1, 0)),
            new Pose2d(-2, -1, new Rotation2d(Math.PI / 3)));

    // Driving groups
    addCommands(
        toReef
            .andThen(new ElevAPIDCmd(elevator, ElevatorConstants.kElevL4Ht))
            .andThen(new AutoSleepCmd(0.5))
            .andThen(new AutoCoralSpinReverseCmd(intake, 1))
            .andThen(new ElevAPIDCmd(elevator, ElevatorConstants.kElevLowHt, 4))
            .andThen(recenter));
  }
}
