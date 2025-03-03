// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.algaeArm;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.AlgaeSpinnerSubsystem;

/** An example command that uses an example subsystem. */
public class AlgaeSpinnerForwardCommand extends Command {
  private final AlgaeSpinnerSubsystem algaeArm_subsystem;

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public AlgaeSpinnerForwardCommand(AlgaeSpinnerSubsystem subsystem) {
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
    algaeArm_subsystem.algaeSpinnerForwardCmd();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
