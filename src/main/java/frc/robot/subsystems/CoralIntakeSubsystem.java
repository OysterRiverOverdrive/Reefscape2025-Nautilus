// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.RobotConstants;

public class CoralIntakeSubsystem extends SubsystemBase {

  private SparkMax Motor = new SparkMax(RobotConstants.kCoralIntakeCanId, MotorType.kBrushless);

  private final SparkMax linActuator =
      new SparkMax(RobotConstants.kCoralActuatorCanId, MotorType.kBrushed);

  private SparkMaxConfig m_actuConfig;

  public CoralIntakeSubsystem() {
    // Digital Configuration for later
    m_actuConfig = new SparkMaxConfig();
    m_actuConfig.inverted(RobotConstants.kCoralActuInverted);
    m_actuConfig.smartCurrentLimit(RobotConstants.kCoralActuCurrentLimit);
    linActuator.configure(
        m_actuConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  public void spinActuUp() {
    linActuator.set(1);
  }

  public void spinActuDown() {
    linActuator.set(-1);
  }

  public void spinActuStop() {
    linActuator.stopMotor();
  }

  public void coralIntakeForwardCmd() {
    Motor.set(RobotConstants.kCoralIntakeSpeed);
  }

  public void coralIntakeReverseCmd() {
    Motor.set(-1 * RobotConstants.kCoralIntakeSpeed);
  }

  public void coralIntakeStopCmd() {
    Motor.stopMotor();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
