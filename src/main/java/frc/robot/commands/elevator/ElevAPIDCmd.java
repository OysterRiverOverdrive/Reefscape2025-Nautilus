// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.elevator;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.PIDConstants;
import frc.robot.subsystems.ElevatorSubsystem;

public class ElevAPIDCmd extends Command {
  private ElevatorSubsystem elevator;
  private double setpoint;
  private double error;
  private double priorSet;
  private boolean rising;

  private final ProfiledPIDController elevatorRisePID =
      new ProfiledPIDController(
          PIDConstants.kElevatorRP,
          PIDConstants.kElevatorRI,
          PIDConstants.kElevatorRD,
          new TrapezoidProfile.Constraints(
              PIDConstants.kElevatorRMaxV, PIDConstants.kElevatorRMaxA));

  private final ProfiledPIDController elevatorBasePID =
      new ProfiledPIDController(
          PIDConstants.kElevatorBP,
          PIDConstants.kElevatorBI,
          PIDConstants.kElevatorBD,
          new TrapezoidProfile.Constraints(
              PIDConstants.kElevatorBMaxV, PIDConstants.kElevatorBMaxA));

  /**
   * Autonomous Method of moving elevator
   *
   * @param elevators instance of elevator subsystem
   * @param setpoint (inches) set height (use elevator constants)
   * @param error (±) range around setpoint that command will end
   */
  public ElevAPIDCmd(ElevatorSubsystem elevator, double setpoint, double error) {
    this.elevator = elevator;
    this.setpoint = setpoint;
    this.error = error;
    addRequirements(elevator);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    priorSet = elevator.getHeight();
    if (setpoint > priorSet) {
      rising = true;
      elevatorRisePID.setGoal(setpoint);
    } else {
      rising = false;
      elevatorBasePID.setGoal(setpoint);
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double elevatorSpeed;
    if (rising) {
      elevatorSpeed = elevatorRisePID.calculate(elevator.getHeight());
    } else {
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
    if (elevator.getHeight() >= (setpoint - error) && elevator.getHeight() <= (setpoint + error)) {
      return true;
    } else {
      return false;
    }
  }
}
