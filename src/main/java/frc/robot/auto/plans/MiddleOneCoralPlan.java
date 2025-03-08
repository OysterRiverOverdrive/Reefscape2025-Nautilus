package frc.robot.auto.plans;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.Constants.RobotConstants.ElevatorConstants;
import frc.robot.auto.AutoCoralSpinForwardCmd;
import frc.robot.auto.AutoCreationCmd;
import frc.robot.auto.AutoSleepCmd;
import frc.robot.commands.elevator.ElevAPIDCmd;
import frc.robot.subsystems.CoralIntakeSubsystem;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import java.util.List;

// STARTS ON THE RIGHT SIDE

public class MiddleOneCoralPlan extends ParallelCommandGroup {

  public MiddleOneCoralPlan(
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
            List.of(new Translation2d(0.5, 0.01)),
            new Pose2d(1.15, .02, new Rotation2d(0)));
    // Place coral

    // Driving groups
    addCommands(
        centerDrive
            .andThen(new ElevAPIDCmd(elevator, ElevatorConstants.kElevL4Ht, 0.5))
            .andThen(new AutoSleepCmd(4))
            .andThen(new AutoCoralSpinForwardCmd(intake, 0.5)));
  }
}
