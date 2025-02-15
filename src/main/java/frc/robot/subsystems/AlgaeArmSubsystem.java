// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.*;
import frc.robot.Constants.RobotConstants.AlgaeArmConstants;

public class AlgaeArmSubsystem extends SubsystemBase {
  /** Creates a new ExampleSubsystem. */
  private final SparkMax m_AlgaeArmSparkMax =
      new SparkMax(RobotConstants.kAlgaeArmCanId, MotorType.kBrushless);

  private final SparkMax m_AlgaeSpinnerSparkMax =
      new SparkMax(RobotConstants.kAlgaeSpinnerCanId, MotorType.kBrushless);

  private final RelativeEncoder m_AlgaeArmEncoder;

  private double elevatorSpeed;

  private final PIDController algaeArmPID =
      new PIDController(PIDConstants.kAlgaeArmP, PIDConstants.kAlgaeArmI, PIDConstants.kAlgaeArmD);

  private double algaeArmPIDSetPoint;

  public AlgaeArmSubsystem() {
    m_AlgaeArmEncoder = m_AlgaeArmSparkMax.getEncoder();
  }

  public void algaeSpinnerForwardCmd() {
    m_AlgaeSpinnerSparkMax.set(RobotConstants.kAlgaeSpinnerSpeed);
  }

  public void algaeSpinnerReverseCmd() {
    m_AlgaeSpinnerSparkMax.set(0 - RobotConstants.kAlgaeSpinnerSpeed);
  }

  public void algaeSpinnerStopCmd() {
    m_AlgaeSpinnerSparkMax.stopMotor();
  }

  public void toDown() {
    algaeArmPIDSetPoint = (AlgaeArmConstants.kAlgaeArmStopRotations[0]);
  }

  public void toFlat() {
    algaeArmPIDSetPoint = (AlgaeArmConstants.kAlgaeArmStopRotations[1]);
  }

  public void toRemoveAlgae() {
    algaeArmPIDSetPoint = (AlgaeArmConstants.kAlgaeArmStopRotations[2]);
  }

  public void toUp() {
    algaeArmPIDSetPoint = (AlgaeArmConstants.kAlgaeArmStopRotations[3]);
  }

  public double getRelativeRotation() {
    return (m_AlgaeArmEncoder.getPosition() * AlgaeArmConstants.kAlgaeArmGearRatio);
  }

  public double getSetPoint() {
    return algaeArmPIDSetPoint;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    elevatorSpeed =
        algaeArmPID.calculate(
            m_AlgaeArmEncoder.getPosition() * AlgaeArmConstants.kAlgaeArmGearRatio);
    m_AlgaeArmSparkMax.set(elevatorSpeed);
    algaeArmPID.setSetpoint(algaeArmPIDSetPoint);
  }
}
