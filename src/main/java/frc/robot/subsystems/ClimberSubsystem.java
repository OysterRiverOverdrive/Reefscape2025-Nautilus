// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkAbsoluteEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.RobotConstants;

public class ClimberSubsystem extends SubsystemBase {

  private SparkMax climber = new SparkMax(RobotConstants.kClimberCanId, MotorType.kBrushless);
  private final SparkAbsoluteEncoder climberAbsEnc;
  private final SparkMaxConfig climberConfig;

  public ClimberSubsystem() {
    climberConfig = new SparkMaxConfig();
    climber.configure(
        climberConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    climberAbsEnc = climber.getAbsoluteEncoder();
  }

  public void climbUpCmd() {
    if (getClimberRot() > RobotConstants.kClimberMaxRotIn) {
      climber.stopMotor();
    } else if (getClimberRot() > RobotConstants.kClimberSlowRot) {
      climber.set(RobotConstants.kClimberSpeedLow);
    } else {
      climber.set(RobotConstants.kClimberSpeedHigh);
    }
  }

  public void climbDownCmd() {
    if (getClimberRot() < RobotConstants.kClimberMaxRotOut) {
      climber.stopMotor();
    } else if (getClimberRot() < RobotConstants.kClimberSlowRot) {
      climber.set(-1 * RobotConstants.kClimberSpeedHigh);
    } else {
      climber.set(-1 * RobotConstants.kClimberSpeedLow);
    }
  }

  public void climberStopCmd() {
    climber.stopMotor();
  }

  public double getClimberRot() {
    return climberAbsEnc.getPosition();
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Climber Rotation", getClimberRot());
  }
}
