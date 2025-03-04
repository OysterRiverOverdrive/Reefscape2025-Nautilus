// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

  public static final class AutoConstants {

    public static final double kMaxSpeedMetersPerSecond = 4.8 / 2;
    public static final double kMaxAngularSpeedRadiansPerSecond = (2 * Math.PI) / 4;
    public static final double kMaxAccelerationMetersPerSecondSquared = 6;
    public static final double kMaxAngularAccelerationRadiansPerSecondSquared = Math.PI / 2;
    public static final double kPXController = 8;
    public static final double kPYController = 8;
    public static final double kPThetaController = 5;

    public static final TrapezoidProfile.Constraints kThetaControllerConstraints =
        new TrapezoidProfile.Constraints(
            kMaxAngularSpeedRadiansPerSecond, kMaxAngularAccelerationRadiansPerSecondSquared);
  }

  // Constants specifically for Driving & Operation
  public static class DriveConstants {
    // Controller Ports ---
    // Determined here but assigned in the driver station to determine and organize physical ports
    // the controllers are plug into
    public static final int kDrveControllerPort = 0;
    public static final int kOperControllerPort = 1;

    // Driver Controller Joystick ---
    public static final int kDriveX = 0;
    public static final int kDriveY = 1;
    public static final int kDriveRotate = 4;
    public static final double deadzoneDriver = 0.12;

    public enum joysticks {
      DRIVER,
      OPERATOR
    }

    // Driving Parameters - Note that these are not the maximum capable speeds of
    // the robot, rather the allowed maximum speeds

    public static final double kMaxSpeedMetersPerSecond = 6.8;
    public static final double kMaxAngularSpeed = 1.25 * 2 * Math.PI; // radians per second

    public static final double kDirectionSlewRate = 4; // radians per second
    public static final double kMagnitudeSlewRate = 2; // percent per second (1 = 100%)
    public static final double kRotationalSlewRate = 5; // percent per second (1 = 100%)

    // Chassis configuration
    public static final double kTrackWidth = Units.inchesToMeters(26.5);
    // Distance between centers of right and left wheels on robot
    public static final double kWheelBase = Units.inchesToMeters(26.5);
    // Distance between front and back wheels on robot
    public static final SwerveDriveKinematics kDriveKinematics =
        new SwerveDriveKinematics(
            new Translation2d(kWheelBase / 2, kTrackWidth / 2),
            new Translation2d(kWheelBase / 2, -kTrackWidth / 2),
            new Translation2d(-kWheelBase / 2, kTrackWidth / 2),
            new Translation2d(-kWheelBase / 2, -kTrackWidth / 2));
  }

  // Constants specifically for the physical robot
  public static final class RobotConstants {
    // SPARK MAX CAN IDs
    public static final int kFrontRightDrivingCanId = 3;
    public static final int kFrontRightTurningCanId = 2;

    public static final int kFrontLeftDrivingCanId = 5;
    public static final int kFrontLeftTurningCanId = 4;

    public static final int kRearRightDrivingCanId = 7;
    public static final int kRearRightTurningCanId = 6;

    public static final int kRearLeftDrivingCanId = 9;
    public static final int kRearLeftTurningCanId = 8;

    public static final int kElevator1CanId = 10;
    public static final int kElevator2CanId = 11;

    public static final int kCoralIntakeCanId = 12;
    public static final int kCoralActuatorCanId = 15;

    public static final int kAlgaeArmCanId = 13;
    public static final int kAlgaeSpinnerCanId = 14;

    // Used to declare Navx as upside down
    public static final boolean kGyroReversed = true;

    // Angular offsets of the modules relative to the chassis in radians
    public static final double kFrontLeftChassisAngularOffset = -Math.PI / 2;
    public static final double kFrontRightChassisAngularOffset = 0;
    public static final double kBackLeftChassisAngularOffset = Math.PI;
    public static final double kBackRightChassisAngularOffset = Math.PI / 2;

    // Algae Spinner constants
    public static final double kAlgaeSpinnerSpeed = 0.45;

    // Coral Intake constants
    public static final double kCoralIntakeSpeed = 0.4;
    public static final boolean kCoralActuInverted = true;
    public static final int kCoralActuCurrentLimit = 1; // Amps

    // Actuator Rates
    public static final double kCoralActuSpeedRate = 0.6; // Inches per second
    public static final double kCoralActuDistance = 2.3; // Inches

    public static final class AlgaeArmConstants {
      // Algae Arm gear ratio, 15 to 1, so motor rotations to arm rotations is 1 to 15
      public static final double kAlgaeArmGearRatio = 1.0 / 15.0;

      // Bypassing Calculated Values with collected values, this can be faster during testing to
      // just record values and then make the calculation work later
      // Later Calculations ------------------------------------------------------
      // Following angles are in degrees relative to bottom.
      // public static final double kAlgaeArmBottomAngle = 0;
      // public static final double kAlgaeArmFlatAngle = 40;
      // public static final double kAlgaeArmRemoveAlgaeAngle = 60;
      // public static final double kAlgaeArmTopAngle = 80;
      // // guess??? maybe? not accurate check when algae arm attatched

      // // Array of angles of algae arm stops relative to base angle in degrees
      // // Indicies are stops in order from bottom to top
      // public static final double[] kAlgaeArmStopAngles = {
      //   AlgaeArmConstants.kAlgaeArmBottomAngle,
      //   AlgaeArmConstants.kAlgaeArmFlatAngle,
      //   AlgaeArmConstants.kAlgaeArmRemoveAlgaeAngle,
      //   AlgaeArmConstants.kAlgaeArmTopAngle
      // };

      // // Same array in terms of rotations
      // public static final double[] kAlgaeArmStopRotations = {
      //   AlgaeArmConstants.kAlgaeArmBottomAngle / 360,
      //   AlgaeArmConstants.kAlgaeArmFlatAngle / 360,
      //   AlgaeArmConstants.kAlgaeArmRemoveAlgaeAngle / 360,
      //   AlgaeArmConstants.kAlgaeArmTopAngle / 360
      // };

      // Bypassing Calculated Values with collected values, this can be faster during testing to
      // just record values and then make the calculation work later
      public static final double kDownAngle = 0.03;
      public static final double kAlgaeAngle = 0.295;
      public static final double kLoadingAngle = 0.169;
    }

    public static final class ElevatorConstants {

      public static final double kElevatorHeightToRot = 10;
      // guess??? maybe? not accurate check when elevator attatched

      // Array of heights of elevator stops relative to base height in inches
      // These hieghts are of the hinge for the coral bucket,
      // recieved by taking Reef Level height,
      // adding height difference between bottom and top of the coral bucket,
      // and subtracting the lowest height reachable by the elevator,
      // making all heights relative to the lowest point on the elevator
      // After, it adds the in the offset so the coral center is lined up
      // Indicies 1-4 are corresponding reef levels, index 0 is base
      // Index 5 is height of elevator for coral intake

      // Bypassing Calculated Values with collected values, this can be faster during testing to
      // just record values and then make the calculation work later
      // Later Calculations ------------------------------------------------------
      // public static final double[] kElevatorStopsCalculated = {
      //   0,
      //   ElevatorConstants.kElevatorReefL1Height
      //       + ElevatorConstants.kElevatorCoralBucketHeightDifference
      //       - ElevatorConstants.kElevatorLowestHeight
      //       + ElevatorConstants.kElevatorReefOffset,
      //   ElevatorConstants.kElevatorReefL2Height
      //       + ElevatorConstants.kElevatorCoralBucketHeightDifference
      //       - ElevatorConstants.kElevatorLowestHeight
      //       + ElevatorConstants.kElevatorReefOffset,
      //   ElevatorConstants.kElevatorReefL3Height
      //       + ElevatorConstants.kElevatorCoralBucketHeightDifference
      //       - ElevatorConstants.kElevatorLowestHeight
      //       + ElevatorConstants.kElevatorReefOffset,
      //   ElevatorConstants.kElevatorReefL4Height
      //       + ElevatorConstants.kElevatorCoralBucketHeightDifference
      //       - ElevatorConstants.kElevatorLowestHeight
      //       + ElevatorConstants.kElevatorReefOffset,
      //   ElevatorConstants.kElevatorIntakeHeight - ElevatorConstants.kElevatorLowestHeight
      // };

      // // Heights for elevator as tested, last one is still guess
      // public static final double[] kElevatorStopsTested = {
      //   0 - ElevatorConstants.kElevatorLowestHeight,
      //   26 - ElevatorConstants.kElevatorLowestHeight,
      //   35.5 - ElevatorConstants.kElevatorLowestHeight,
      //   51.5 - ElevatorConstants.kElevatorLowestHeight,
      //   76.5 - ElevatorConstants.kElevatorLowestHeight,
      //   25 - ElevatorConstants.kElevatorLowestHeight
      // };

      // All measurements are from ground to the top blue beam on the carriage
      // Lowest Height is confirmed, rest are guesses

      public static final double kElevLowHt = 18;
      public static final double kElevL1Ht = 22;
      public static final double kElevL2Ht = 32.5;
      public static final double kElevL3Ht = 49;
      public static final double kElevL4Ht = 74;
      public static final double kElevIntakeHt = 27.5;
      public static final double kElevatorAboveIntakeHeightDifference = 8;

      public static final double kElevatorReefOffset = -2;
      // Coral bucket downward angle from flat, degrees
      public static final double kElevatorCoralBucketAngleDegrees = 28;
      // Coral bucket downward angle from flat, degrees
      public static final double kElevatorCoralBucketAngleRadians =
          kElevatorCoralBucketAngleDegrees * Math.PI / 180.0;
      // Height difference between pivot point and end of coral bucket for elevator
      public static final double kElevatorCoralBucketHeightDifference =
          8 * Math.sin(kElevatorCoralBucketAngleRadians);

      public static final double[][] ELEV_SAFETY_POINTS = {
        {0.0, 78.0},
        {0.05, 76.0},
        {0.1, 74.0},
        {0.2, 60.0},
        {0.3, 36.0},
        {0.4, 34.0},
        {0.5, 32.0},
        {0.6, 30.0},
        {0.7, 28.0},
        {0.8, 26.0},
        {0.9, 23.0},
        {1.0, 20.0}
      };

      // Polynomial degree (adjust as needed)
      public static final int POLYNOMIAL_DEGREE = 2;
    }
  }

  // Constants for PIDs
  public static final class PIDConstants {

    // Elevator PID
    public static final double kElevatorRP = 0.0068;
    public static final double kElevatorRI = 0.0011;
    public static final double kElevatorRD = 0.0025;

    public static final double kElevatorBP = 0.0078;
    public static final double kElevatorBI = 0.0013;
    public static final double kElevatorBD = 0.0021;

    // Autonomous Elevator PID
    public static final double kAutoElevP = 0.008;
    public static final double kAutoElevI = 0.0013;
    public static final double kAutoElevD = 0.0018;

    // Algae Arm PID
    public static final double kAlgaeArmP = 0.5;
    public static final double kAlgaeArmI = 0.8;
    public static final double kAlgaeArmD = 0;
    // Autonomous Algae Arm PID
    public static final double kAutoAlgaeP = 0.8;
    public static final double kAutoAlgaeI = 0.5;
    public static final double kAutoAlgaeD = 0.1;
  }

  // Constants specifically for Swerve Module
  public static final class ModuleConstants {
    // The MAXSwerve module can be configured with one of three pinion gears: 12T, 13T, or 14T.
    // This changes the drive speed of the module (a pinion gear with more teeth will result in a
    // robot that drives faster).
    public static final int kDrivingMotorPinionTeeth = 13;

    // Invert the turning encoder, since the output shaft rotates in the opposite direction of
    // the steering motor in the MAXSwerve Module.
    public static final boolean kTurningEncoderInverted = true;

    // Calculations required for driving motor conversion factors and feed forward
    public static final double kDrivingMotorFreeSpeedRps = NeoMotorConstants.kFreeSpeedRpm / 60;
    public static final double kWheelDiameterMeters = 0.0762;
    public static final double kWheelCircumferenceMeters = kWheelDiameterMeters * Math.PI;
    // 45 teeth on the wheel's bevel gear, 22 teeth on the first-stage spur gear, 15 teeth on the
    // bevel pinion
    public static final double kDrivingMotorReduction =
        (45.0 * 22) / (kDrivingMotorPinionTeeth * 15);
    public static final double kDriveWheelFreeSpeedRps =
        (kDrivingMotorFreeSpeedRps * kWheelCircumferenceMeters) / kDrivingMotorReduction;
    public static final double kDrivingEncoderPositionFactor =
        (kWheelDiameterMeters * Math.PI) / kDrivingMotorReduction; // meters
    public static final double kDrivingEncoderVelocityFactor =
        ((kWheelDiameterMeters * Math.PI) / kDrivingMotorReduction) / 60.0; // meters per second
    public static final double kTurningEncoderPositionFactor = (2 * Math.PI); // radians
    public static final double kTurningEncoderVelocityFactor =
        (2 * Math.PI) / 60.0; // radians per second
    public static final double kTurningEncoderPositionPIDMinInput = 0; // radians
    public static final double kTurningEncoderPositionPIDMaxInput =
        kTurningEncoderPositionFactor; // radians

    // PID Driving Values ---
    // Most likely used to act as a form of slew
    // See:
    // https://docs.wpilib.org/en/stable/docs/software/advanced-controls/filters/slew-rate-limiter.html
    public static final double kDrivingP = 0.04;
    public static final double kDrivingI = 0;
    public static final double kDrivingD = 0;
    public static final double kDrivingFF = 1 / kDriveWheelFreeSpeedRps;
    public static final double kDrivingMaxOutput = 1;
    public static final double kDrivingMinOutput = kDrivingMaxOutput * -1;
    public static final double kTurningP = 1;
    public static final double kTurningI = 0;
    public static final double kTurningD = 0;
    public static final double kTurningFF = 0;
    public static final double kTurningMaxOutput = 1;
    public static final double kTurningMinOutput = kTurningMaxOutput * -1;

    // Swerve Module Idle Modes ---
    // These determine how the module behavior when there is a lack of input from the driver
    public static final IdleMode kDrivingMotorIdleMode = IdleMode.kBrake;
    public static final IdleMode kTurningMotorIdleMode = IdleMode.kBrake;

    // Current limits ---
    // Meant for the electrical side of the drivetrain to make sure that the drivetrain isn't
    // drawing too much power
    public static final int kDrivingMotorCurrentLimit = 40; // amps
    public static final int kTurningMotorCurrentLimit = 15; // amps
  }

  public static final class NeoMotorConstants {
    public static final double kFreeSpeedRpm = 5676;
  }

  public static final class TimerConstants {
    // from timer from when autonomous starts to autonmous ends
    public static final int AutoTimerLength = 15; // seconds
    // timer that starts when autonomous ends until Halftime
    public static final int TeleOpStartTimerLength = 75; // seconds
    // timer that starts when Halftime untill match ends
    public static final int TeleOpEndTimerLength = 75; // seconds
  }
}
