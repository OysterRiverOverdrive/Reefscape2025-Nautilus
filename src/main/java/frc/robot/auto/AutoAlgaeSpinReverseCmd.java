// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.auto;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.AlgaeSpinnerSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class AutoAlgaeSpinReverseCmd extends Command {
  private AlgaeSpinnerSubsystem algae;
  private Timer timer = new Timer();
  private double duration;

  /** Creates a new AutoAlgaeSpinForwardCmd. */
  public AutoAlgaeSpinReverseCmd(AlgaeSpinnerSubsystem algae, double duration) {
    this.algae = algae;
    this.duration = duration;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(algae);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    timer.reset();
    timer.start();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    algae.algaeSpinnerReverseCmd();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    algae.algaeSpinnerStopCmd();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return timer.hasElapsed(duration) ? true : false;
  }
}
