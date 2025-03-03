// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.algaeArm;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.PIDConstants;
import frc.robot.subsystems.AlgaeArmSubsystem;

public class AlgaeAPIDCmd extends Command {

  private AlgaeArmSubsystem algae;
  private double setpoint;
  private double error;

  private final PIDController algaeArmPID =
      new PIDController(
          PIDConstants.kAutoAlgaeP, PIDConstants.kAutoAlgaeI, PIDConstants.kAutoAlgaeD);

  /**
   * Autonomous Method of moving elevator
   *
   * @param algae instance of elevator subsystem
   * @param setpoint (% of rot) set height (use elevator constants)
   * @param error (±) range around setpoint that command will end
   */
  public AlgaeAPIDCmd(AlgaeArmSubsystem algae, double setpoint, double error) {
    this.algae = algae;
    this.setpoint = setpoint;
    this.error = error;
    addRequirements(algae);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    algaeArmPID.setSetpoint(setpoint);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double armSpeed = algaeArmPID.calculate(algae.getEncoder());
    algae.setAlgaeArmSpeed(armSpeed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (algae.getEncoder() >= (setpoint - error) && algae.getEncoder() <= (setpoint + error)) {
      return true;
    } else {
      return false;
    }
  }
}
