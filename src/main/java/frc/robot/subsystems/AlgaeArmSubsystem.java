// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.*;
import frc.robot.Constants.RobotConstants.AlgaeArmConstants;

public class AlgaeArmSubsystem extends SubsystemBase {

  private final SparkMax m_AlgaeArmSparkMax =
      new SparkMax(RobotConstants.kAlgaeArmCanId, MotorType.kBrushless);

  private final RelativeEncoder m_AlgaeArmEncoder;

  private SparkMaxConfig m_ArmConfig;

  private double algaeArmPIDSetPoint;

  public AlgaeArmSubsystem() {
    m_AlgaeArmEncoder = m_AlgaeArmSparkMax.getEncoder();
    m_AlgaeArmEncoder.setPosition(0);
    m_ArmConfig = new SparkMaxConfig();
    m_ArmConfig.inverted(true);
    m_AlgaeArmSparkMax.configure(
        m_ArmConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  public double getEncoder() {
    return m_AlgaeArmEncoder.getPosition() * AlgaeArmConstants.kAlgaeArmGearRatio;
  }

  public void toDown() {
    algaeArmPIDSetPoint = (AlgaeArmConstants.kDownAngle);
  }

  public void toRemoveAlgae() {
    algaeArmPIDSetPoint = (AlgaeArmConstants.kAlgaeAngle);
  }

  public void toLoad() {
    algaeArmPIDSetPoint = (AlgaeArmConstants.kLoadingAngle);
  }

  public double getRelativeRotation() {
    return (m_AlgaeArmEncoder.getPosition() * AlgaeArmConstants.kAlgaeArmGearRatio);
  }

  public double getSetPoint() {
    return algaeArmPIDSetPoint;
  }

  public void setAlgaeArmSpeed(double speed) {
    m_AlgaeArmSparkMax.set(speed);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Algae Arm Angle", getEncoder());
    SmartDashboard.putNumber("Algae Arm Set", getSetPoint());
  }
}
