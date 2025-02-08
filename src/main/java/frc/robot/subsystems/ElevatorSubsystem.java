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

  private double elevatorSpeed;

  private final PIDController elevatorPID =
      new PIDController(PIDConstants.kElevatorP, PIDConstants.kElevatorI, PIDConstants.kElevatorD);

  private double elevatorPIDSetPoint;

  public ElevatorSubsystem() {

    m_elevator2Config = new SparkMaxConfig();

    m_elevator1Encoder = m_elevator1SparkMax.getAbsoluteEncoder();

    m_elevator2Config.follow(m_elevator1SparkMax);

    m_elevator2SparkMax.configure(
        m_elevator2Config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  public void toBase() {
    elevatorPIDSetPoint = (ElevatorConstants.kElevatorStopsTested[0]);
  }

  public void toL1() {
    elevatorPIDSetPoint = (ElevatorConstants.kElevatorStopsTested[1]);
  }

  public void toL2() {
    elevatorPIDSetPoint = (ElevatorConstants.kElevatorStopsTested[2]);
  }

  public void toL3() {
    elevatorPIDSetPoint = (ElevatorConstants.kElevatorStopsTested[3]);
  }

  public void toL4() {
    elevatorPIDSetPoint = (ElevatorConstants.kElevatorStopsTested[4]);
  }

  public void toIntake() {
    elevatorPIDSetPoint = (ElevatorConstants.kElevatorStopsTested[5]);
  }

  public void toAboveIntake() {
    elevatorPIDSetPoint =
        (ElevatorConstants.kElevatorStopsTested[5]
            + ElevatorConstants.kElevatorAboveIntakeHeightDifference);
  }

  public double getHeight() {
    return ((m_elevator1Encoder.getPosition() * ElevatorConstants.kElevatorHeightToRot)
        + ElevatorConstants.kElevatorLowestHeight);
  }

  public double getRelativeHeight() {
    return (m_elevator1Encoder.getPosition() * ElevatorConstants.kElevatorHeightToRot);
  }

  public double getSetPoint() {
    return elevatorPIDSetPoint;
  }

  // Logs print encoder and corresponding elevator height values
  // Possible print method for heightGet:
  /*
  System.out.println("Encoder: " + m_elevator1Encoder.getPosition() + " rotations");
  System.out.println(
      "Elevator Relative Height: "
          + (m_elevator1Encoder.getPosition() * ElevatorConstants.kElevatorHeightToRot)
          + " inches");
  System.out.println(
      "Elevator Height: "
          + ((m_elevator1Encoder.getPosition() * ElevatorConstants.kElevatorHeightToRot)
              + ElevatorConstants.kElevatorLowestHeight)
          + " inches"); */

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    elevatorSpeed =
        elevatorPID.calculate(
            m_elevator1Encoder.getPosition() * ElevatorConstants.kElevatorHeightToRot);
    m_elevator1SparkMax.set(elevatorSpeed);
    elevatorPID.setSetpoint(elevatorPIDSetPoint);

    SmartDashboard.putNumber("Elevator Height", getHeight());
  }
}
