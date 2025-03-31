// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.climber;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.RobotConstants;
import frc.robot.subsystems.ClimberSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class climbUpCmd extends Command {
  /** Creates a new climbUpCmd. */
  private ClimberSubsystem climber;

  public climbUpCmd(ClimberSubsystem climber) {
    this.climber = climber;
    addRequirements(climber);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    climber.climbUpCmd();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    climber.climberStopCmd();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (climber.getClimberRot() > RobotConstants.kClimberMaxRotIn) {
      return true;
    } else {
      return false;
    }
  }
}
