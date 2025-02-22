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
import frc.robot.Constants.*;
import frc.robot.Constants.RobotConstants.ElevatorConstants;

public class ElevatorSubsystem extends SubsystemBase {
  /** Creates a new ExampleSubsystem. */
  private final SparkMax m_elevator1SparkMax =
      new SparkMax(RobotConstants.kElevator1CanId, MotorType.kBrushless);

  private final SparkMax m_elevator2SparkMax =
      new SparkMax(RobotConstants.kElevator2CanId, MotorType.kBrushless);

  private final SparkAbsoluteEncoder m_elevator1Encoder;

  private final SparkMaxConfig m_elevator1Config;
  private final SparkMaxConfig m_elevator2Config;

  // Logic Variables
  private double elevatorPIDSetPoint;
  private double encoderRot;
  private double prevRot;
  private double rot;
  private double rotcount = 0;

  public ElevatorSubsystem() {

    // Leading motor config
    m_elevator1Config = new SparkMaxConfig();
    m_elevator1Config.inverted(true);
    m_elevator1Config.absoluteEncoder.positionConversionFactor(30).inverted(true);
    m_elevator1SparkMax.configure(
        m_elevator1Config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    // Slave motor config
    m_elevator2Config = new SparkMaxConfig();
    m_elevator2Config.follow(m_elevator1SparkMax);
    m_elevator2Config.inverted(true);
    m_elevator2SparkMax.configure(
        m_elevator2Config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    m_elevator1Encoder = m_elevator1SparkMax.getAbsoluteEncoder();
    prevRot = m_elevator1Encoder.getPosition();

    // Set starting height to bottom height
    toBase();
  }

  public double getEncoder() {
    // Rotations Translated to Total Distance Travelled
    return ((rotcount * 30) + m_elevator1Encoder.getPosition());
  }

  public double getHeight() {
    // Function to convert Encoder to Carriage Height approximately in inches
    return (0.555 * getEncoder() + 17.7);
  }

  public void toBase() {
    elevatorPIDSetPoint = (ElevatorConstants.kElevLowHt);
  }

  public void toL1() {
    elevatorPIDSetPoint = (ElevatorConstants.kElevL1Ht);
  }

  public void toL2() {
    elevatorPIDSetPoint = (ElevatorConstants.kElevL2Ht);
  }

  public void toL3() {
    elevatorPIDSetPoint = (ElevatorConstants.kElevL3Ht);
  }

  public void toL4() {
    elevatorPIDSetPoint = (ElevatorConstants.kElevL4Ht);
  }

  public void toIntake() {
    elevatorPIDSetPoint = (ElevatorConstants.kElevIntakeHt);
  }

  public void toAboveIntake() {
    elevatorPIDSetPoint =
        (ElevatorConstants.kElevIntakeHt + ElevatorConstants.kElevatorAboveIntakeHeightDifference);
  }

  public double getSetPoint() {
    return elevatorPIDSetPoint;
  }

  public void setElevatorSpeed(double speed) {
    m_elevator1SparkMax.set(speed);
  }

  @Override
  public void periodic() {
    // Logic to track looping encoder
    rot = m_elevator1Encoder.getPosition();
    if ((rot - prevRot) < -25) {
      rotcount += 1;
    } else if ((rot - prevRot) > 25) {
      rotcount -= 1;
    }
    prevRot = rot;

    SmartDashboard.putNumber("Elev Height", getHeight());
    SmartDashboard.putNumber("Elev Setpoint", elevatorPIDSetPoint);
  }
}
