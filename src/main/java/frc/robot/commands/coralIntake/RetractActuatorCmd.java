// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.coralIntake;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.RobotConstants;
import frc.robot.subsystems.CoralIntakeSubsystem;

public class RetractActuatorCmd extends Command {
  private CoralIntakeSubsystem coral;
  private Timer timer = new Timer();
  // t = d/v
  private double runtime = RobotConstants.kCoralActuDistance / RobotConstants.kCoralActuSpeedRate;

  public RetractActuatorCmd(CoralIntakeSubsystem coral) {
    this.coral = coral;
    addRequirements(coral);
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
    coral.spinActuDown();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    coral.spinActuStop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    boolean retVal = false;
    double currTime = timer.get();
    if (currTime >= runtime) {
      retVal = true;
    }

    return retVal;
  }
}
