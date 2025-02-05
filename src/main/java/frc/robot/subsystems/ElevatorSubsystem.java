// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.*;
import frc.robot.Constants.RobotConstants.ElevatorConstants;

public class ElevatorSubsystem extends SubsystemBase {
  /** Creates a new ExampleSubsystem. */
  private final SparkMax m_elevator1SparkMax =
      new SparkMax(RobotConstants.kElevator1CanId, MotorType.kBrushless);

  private final SparkMax m_elevator2SparkMax =
      new SparkMax(RobotConstants.kElevator2CanId, MotorType.kBrushless);

  private final AbsoluteEncoder m_elevator1Encoder;

  private final SparkMaxConfig m_elevator2Config;

  private final PIDController elevatorPID =
      new PIDController(PIDConstants.kElevatorP, PIDConstants.kElevatorI, PIDConstants.kElevatorD);

  private double setpoint;
  private double safetysetpoint;

  public ElevatorSubsystem() {

    m_elevator2Config = new SparkMaxConfig();

    m_elevator1Encoder = m_elevator1SparkMax.getAbsoluteEncoder();

    m_elevator2Config.follow(m_elevator1SparkMax);

    m_elevator2SparkMax.configure(
        m_elevator2Config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  public void toBase() {
    setpoint = ElevatorConstants.kElevatorStopsTested[0];
  }

  public void toL1() {
    setpoint = ElevatorConstants.kElevatorStopsTested[1];
  }

  public void toL2() {
    setpoint = ElevatorConstants.kElevatorStopsTested[2];
  }

  public void toL3() {
    setpoint = ElevatorConstants.kElevatorStopsTested[3];
  }

  public void toL4() {
    setpoint = ElevatorConstants.kElevatorStopsTested[4];
  }

  public void toIntake() {
    setpoint = ElevatorConstants.kElevatorStopsTested[5];
  }

  // Logs print encoder and corresponding elevator height values
  public double getHeight() {
    return ((m_elevator1Encoder.getPosition() * ElevatorConstants.kElevatorHeightToRot)
        + ElevatorConstants.kElevatorLowestHeight);
  }

  // Calculated equation based on demo speeds and heights
  // Sheet used for calculation in software drive
  public double safetyheight(double x) {
    return (79.1
        + 28.4 * x
        + -725 * Math.pow(x, 2)
        + 1160 * Math.pow(x, 3)
        + -536 * Math.pow(x, 4));
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    double drivespeed = DrivetrainSubsystem.maxSpeedCmd;

    safetysetpoint = safetyheight(drivespeed);
    double location;
    if (safetysetpoint < setpoint) {
      location = safetysetpoint;
    } else {
      location = setpoint;
    }

    elevatorPID.setSetpoint(location);
    double elevatorSpeed =
        elevatorPID.calculate(
            m_elevator1Encoder.getPosition() * ElevatorConstants.kElevatorHeightToRot);
    m_elevator1SparkMax.set(elevatorSpeed);

    SmartDashboard.putNumber("Elevator Height", getHeight());
  }
}
