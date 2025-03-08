// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.elevator;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.PIDConstants;
import frc.robot.subsystems.ElevatorSubsystem;

public class ElevTPIDCmd extends Command {
  private ElevatorSubsystem elevator;
  private double safetysetpoint; // Calculated Max Height
  private double location;

  private final ProfiledPIDController elevatorRisePID = 
        new ProfiledPIDController(
            PIDConstants.kElevatorRP,
            PIDConstants.kElevatorRI,
            PIDConstants.kElevatorRD,
            new TrapezoidProfile.Constraints(PIDConstants.kElevatorRMaxV, PIDConstants.kElevatorRMaxA));

  private final ProfiledPIDController elevatorBasePID = 
        new ProfiledPIDController(
            PIDConstants.kElevatorBP,
            PIDConstants.kElevatorBI,
            PIDConstants.kElevatorBD,
            new TrapezoidProfile.Constraints(PIDConstants.kElevatorBMaxV, PIDConstants.kElevatorBMaxA));

  public ElevTPIDCmd(ElevatorSubsystem elevator) {
    this.elevator = elevator;
    addRequirements(elevator);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    safetysetpoint = elevator.safetyheight();
    if (safetysetpoint < elevator.getSetPoint()) {
      location = safetysetpoint;
      elevator.safetyActive = true;
    } else {
      location = elevator.getSetPoint();
      elevator.safetyActive = false;
    }
    double elevatorSpeed;
    if (elevator.getPIDDir()) {
      elevatorRisePID.setGoal(location);
      elevatorSpeed = elevatorRisePID.calculate(elevator.getHeight());
    } else {
      elevatorBasePID.setGoal(location);
      elevatorSpeed = elevatorBasePID.calculate(elevator.getHeight());
    }

    elevator.setElevatorSpeed(elevatorSpeed);
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
