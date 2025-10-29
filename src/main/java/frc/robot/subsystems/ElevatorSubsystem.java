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
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.*;
import frc.robot.Constants.RobotConstants.ElevatorConstants;
import java.util.ArrayList;
import org.apache.commons.math3.analysis.polynomials.PolynomialFunction;
import org.apache.commons.math3.fitting.PolynomialCurveFitter;
import org.apache.commons.math3.fitting.WeightedObservedPoints;

public class ElevatorSubsystem extends SubsystemBase {
  /** Creates a new ExampleSubsystem. */
  private final SparkMax m_elevator1SparkMax =
      new SparkMax(RobotConstants.kElevator1CanId, MotorType.kBrushless);

  private final SparkMax m_elevator2SparkMax =
      new SparkMax(RobotConstants.kElevator2CanId, MotorType.kBrushless);

  private final SparkAbsoluteEncoder m_elevator1Encoder;

  private final SparkMaxConfig m_elevator1Config;
  private final SparkMaxConfig m_elevator2Config;

  private DrivetrainSubsystem drivetrain;

  // Logic Variables
  private double elevatorPIDSetPoint; // Elevator commanded setpoint
  private double prevRot; // prior enc value for comparison
  private double rot; // enc
  private double rotcount = 0; // # of rotations completed by abs enc
  private final PolynomialFunction polynomial; // Max Height Function
  public boolean safetyActive = false; // Bool for dashboard on height override
  public boolean activePID = true; // Bool for safety override on PID

  // List of past encoder values to detect encoder disconnection
  private ArrayList<Double> pastEncoderValues = new ArrayList<>();

  // Boolean for whether the encoder is connected
  private boolean encoderConnected = true;

  // PID Network Table
  private static final NetworkTableInstance inst = NetworkTableInstance.getDefault();
  private static final NetworkTable table = inst.getTable("ElevatorPID");
  private static final NetworkTableEntry timestampEntry = table.getEntry("Timestamp");
  private static final NetworkTableEntry setpointEntry = table.getEntry("Setpoint");
  private static final NetworkTableEntry actualEntry = table.getEntry("ActualValue");

  public final ProfiledPIDController elevatorPID =
      new ProfiledPIDController(
          PIDConstants.kElevatorRP,
          PIDConstants.kElevatorRI,
          PIDConstants.kElevatorRD,
          new TrapezoidProfile.Constraints(
              PIDConstants.kElevatorRMaxV, PIDConstants.kElevatorRMaxA));

  public ElevatorSubsystem(DrivetrainSubsystem drivetrain) {
    WeightedObservedPoints elevSafetyPoints = new WeightedObservedPoints();
    // Load points from Constants
    for (double[] point : Constants.RobotConstants.ElevatorConstants.ELEV_SAFETY_POINTS) {
      elevSafetyPoints.add(point[0], point[1]);
    }
    this.drivetrain = drivetrain;

    // Fit polynomial of defined degree
    PolynomialCurveFitter fitter =
        PolynomialCurveFitter.create(Constants.RobotConstants.ElevatorConstants.POLYNOMIAL_DEGREE);
    double[] coefficients = fitter.fit(elevSafetyPoints.toList());

    // Store polynomial function
    polynomial = new PolynomialFunction(coefficients);
    System.out.println("Polynomial Equation: " + polynomial);

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

    // Adding junk values to fill list with 5 values
    pastEncoderValues.add(-1000000.0);
    pastEncoderValues.add(-2000000.0);
    pastEncoderValues.add(-3000000.0);
    pastEncoderValues.add(-4000000.0);
    pastEncoderValues.add(-5000000.0);
  }

  public double getEncoder() {
    // Rotations Translated to Total Distance Travelled
    return ((rotcount * 30) + m_elevator1Encoder.getPosition());
  }

  public double getHeight() {
    // Function to convert Encoder to Carriage Height approximately in inches
    return (0.548 * getEncoder() + 17.6);
  }

  // Calculated equation based on demo speeds and heights
  // Sheet used for calculation in software drive
  public double safetyheight() {
    double drivespeed = drivetrain.maxSpeedCmd;
    return polynomial.value(drivespeed);
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

  public double getSetPoint() {
    return elevatorPIDSetPoint;
  }

  public void setSetPoint(double value) {
    elevatorPIDSetPoint = value;
  }

  public void setElevatorSpeed(double speed) {
    m_elevator1SparkMax.set(speed);
  }

  public void setElevatorStopped() {
    m_elevator1SparkMax.stopMotor();
  }

  public void toggleElevPID() {
    activePID = !activePID; // Invert PID Override
  }

  public void overridePID() {
    activePID = false; // Override PID, ignoring current override state
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    // Logic to track looping encoder
    rot = m_elevator1Encoder.getPosition();
    if ((rot - prevRot) < -25) {
      rotcount += 1;
    } else if ((rot - prevRot) > 25) {
      rotcount -= 1;
    }
    prevRot = rot;

    // PID Network Table
    double timestamp = Timer.getFPGATimestamp(); // Get current time in seconds
    timestampEntry.setDouble(timestamp);
    setpointEntry.setDouble(elevatorPIDSetPoint);
    actualEntry.setDouble(getHeight());

    // PID failsafe
    pastEncoderValues.remove(0); // Remove oldest value
    pastEncoderValues.add(m_elevator1Encoder.getPosition()); // Add current value at the end
    // If all five past encoder values are equal, then the encoder is not giving updated values,
    // and is disconnected in some way, so the PID is overridden
    if ((pastEncoderValues.get(0).equals(pastEncoderValues.get(1)))
        && (pastEncoderValues.get(0).equals(pastEncoderValues.get(2)))
        && (pastEncoderValues.get(0).equals(pastEncoderValues.get(3)))
        && (pastEncoderValues.get(0).equals(pastEncoderValues.get(4)))) {
      overridePID();
      // The encoder is disconnected
      encoderConnected = false;
    } else {
      encoderConnected = true;
    }

    SmartDashboard.putBoolean("Safety Active", safetyActive);
    SmartDashboard.putNumber("Elev Height", getHeight());
    SmartDashboard.putNumber("Elev Setpoint", elevatorPIDSetPoint);
    SmartDashboard.putBoolean("PID Overrided", !activePID); // Inverted for clarity
    SmartDashboard.putBoolean("Elev Connect", encoderConnected);
    SmartDashboard.putBoolean(
        "Elev Connect 2",
        (pastEncoderValues.get(0).equals(pastEncoderValues.get(1)))
            && (pastEncoderValues.get(0).equals(pastEncoderValues.get(2))));
    SmartDashboard.putBoolean(
        "Elev Connect 3",
        Math.abs(pastEncoderValues.get(0) - pastEncoderValues.get(1)) < 0.0000001
            && Math.abs(pastEncoderValues.get(0) - pastEncoderValues.get(2)) < 0.0000001);
    SmartDashboard.putNumber("Elev Past 1", pastEncoderValues.get(0));
    SmartDashboard.putNumber("Elev Past 2", pastEncoderValues.get(1));
    SmartDashboard.putNumber("Elev Past 3", pastEncoderValues.get(2));
  }
}
