// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.RobotConstants;

public class ClimberSubsystem extends SubsystemBase {

  private SparkMax Climber = new SparkMax(RobotConstants.kClimberCanId, MotorType.kBrushless);

  public ClimberSubsystem() {}

  public void climberForwardCmd() {
    Climber.set(RobotConstants.kClimberSpeed);
  }

  public void climberReverseCmd() {
    Climber.set(-1 * RobotConstants.kClimberSpeed);
  }

  public void climberStopCmd() {
    Climber.stopMotor();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
