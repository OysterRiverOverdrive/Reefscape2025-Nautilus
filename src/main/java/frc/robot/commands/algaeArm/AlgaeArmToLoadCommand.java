// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.algaeArm;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.AlgaeArmSubsystem;

/** An example command that uses an example subsystem. */
public class AlgaeArmToLoadCommand extends Command {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final AlgaeArmSubsystem algaeArm_subsystem;

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public AlgaeArmToLoadCommand(AlgaeArmSubsystem subsystem) {
    algaeArm_subsystem = subsystem;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    algaeArm_subsystem.toLoad();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return (algaeArm_subsystem.getSetPoint() - 0.5 <= algaeArm_subsystem.getRelativeRotation()
        && algaeArm_subsystem.getRelativeRotation() <= algaeArm_subsystem.getSetPoint() + 0.5);
  }
}
