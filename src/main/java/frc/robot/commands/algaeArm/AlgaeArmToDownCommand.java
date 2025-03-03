// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.algaeArm;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.AlgaeArmSubsystem;
import frc.robot.subsystems.AlgaeSpinnerSubsystem;

/** An example command that uses an example subsystem. */
public class AlgaeArmToDownCommand extends Command {
  private final AlgaeArmSubsystem algaeArm_subsystem;
  private final AlgaeSpinnerSubsystem algaeSpinner_subsystem;

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public AlgaeArmToDownCommand(AlgaeArmSubsystem subsystem, AlgaeSpinnerSubsystem spinner) {
    algaeArm_subsystem = subsystem;
    algaeSpinner_subsystem = spinner;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    algaeArm_subsystem.toDown();
    algaeSpinner_subsystem.algaeSpinnerStopCmd();
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
