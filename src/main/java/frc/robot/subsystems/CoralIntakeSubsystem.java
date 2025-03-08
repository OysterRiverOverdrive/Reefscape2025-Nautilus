// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.RobotConstants;

public class CoralIntakeSubsystem extends SubsystemBase {

  private SparkMax Spinner = new SparkMax(RobotConstants.kCoralIntakeCanId, MotorType.kBrushless);

  public CoralIntakeSubsystem() {}

  public void coralIntakeForwardCmd() {
    Spinner.set(RobotConstants.kCoralIntakeSpeed);
  }

  public void coralIntakeReverseCmd() {
    Spinner.set(-1 * RobotConstants.kCoralIntakeSpeed);
  }

  public void coralIntakeStopCmd() {
    Spinner.stopMotor();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
