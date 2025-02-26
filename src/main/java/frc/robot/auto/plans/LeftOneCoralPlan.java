package frc.robot.auto.plans;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.auto.AutoCoralSpinForwardCmd;
import frc.robot.auto.AutoCreationCmd;
import frc.robot.commands.coralIntake.ResetActuatorCmd;
import frc.robot.subsystems.CoralIntakeSubsystem;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.auto.AutoSleepCmd;
import java.util.List;

// STARTS ON THE RIGHT SIDE

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
            List.of(new Translation2d(2, 0.5)),
            new Pose2d(3.75, -0.94, new Rotation2d(-Math.PI * 2 / 3)));
    // Place coral

    // Driving groups
    addCommands(
      new ResetActuatorCmd(intake)
      .andThen(toReef)
      .andThen(new InstantCommand(() -> elevator.toL4()))
      .andThen(new AutoSleepCmd(3))
      .andThen(new AutoCoralSpinForwardCmd(intake,1))
      );
  }
}
