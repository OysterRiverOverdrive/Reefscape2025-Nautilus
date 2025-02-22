package frc.robot.auto.plans;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.subsystems.DrivetrainSubsystem;

public class AutoPlanEx extends ParallelCommandGroup {

  public AutoPlanEx(DrivetrainSubsystem drivetrain
      // ,ElevatorSubsystem elevator
      ) {

    /*

    declare driving schematics

    AutoCreationCmd autodrive = new AutoCreationCmd();

    Ex.
    Command MoveToReef =
    autodrive.AutoDriveCmd(drivetrain, list.of(
    new Pose2d(325.68, 158.515, new Rotation2d(0)),
    new Pose2d(209.49, 158.515, new Rotation2d(0))
    ));

    --Then for formatting event-based commands toegther--

    function andThen() is a sequential command group
        -commands one after another
    function alongWith() is a parallel command group
        -commands with each other
    function deadlineWith() runs commands alongside the previous group until one of the commands stop.
        -the first command to stop will stop the entire group

    first:
    andThen(alongWith(Command... commands that are going to run))
    .andThen(alongWith())
    ...
        .deadlineWith()
    and that's the first cycle :)


    Ex.
    andThen(alongWith(MoveToReef, New AutoElevatorCmd(elevator, ElevatorToLv1)))
    .andThen(New AutoElevatorCmd(elevator, ReleaseCoral))

    **this example code just drives straight to the reef from the middle starting point and unloads a coral onto the reef.

    */
  }
}
