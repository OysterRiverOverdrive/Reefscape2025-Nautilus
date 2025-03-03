// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.RobotConstants;

public class AlgaeSpinnerSubsystem extends SubsystemBase {
  /** Creates a new AlgaeSpinnerSubsystem. */

  private final SparkMax m_AlgaeSpinnerSparkMax =
      new SparkMax(RobotConstants.kAlgaeSpinnerCanId, MotorType.kBrushless);

  private SparkMaxConfig m_SpinnerConfig;

  public AlgaeSpinnerSubsystem() {}

  public void algaeSpinnerForwardCmd() {
    m_AlgaeSpinnerSparkMax.set(RobotConstants.kAlgaeSpinnerSpeed);
  }

  public void algaeSpinnerReverseCmd() {
    m_AlgaeSpinnerSparkMax.set(-1 * RobotConstants.kAlgaeSpinnerSpeed);
  }

  public void algaeSpinnerStopCmd() {
    m_AlgaeSpinnerSparkMax.stopMotor();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
