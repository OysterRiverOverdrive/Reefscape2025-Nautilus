// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.studica.frc.AHRS;
import com.studica.frc.AHRS.NavXComType;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.util.WPIUtilJNI;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.RobotConstants;
import frc.utils.LimelightHelpers;
import frc.utils.SwerveModule;

public class DrivetrainSubsystem extends SubsystemBase {
  // Create SwerveModules
  private final SwerveModule m_frontLeft =
      new SwerveModule(
          RobotConstants.kFrontLeftDrivingCanId,
          RobotConstants.kFrontLeftTurningCanId,
          RobotConstants.kFrontLeftChassisAngularOffset);

  private final SwerveModule m_frontRight =
      new SwerveModule(
          RobotConstants.kFrontRightDrivingCanId,
          RobotConstants.kFrontRightTurningCanId,
          RobotConstants.kFrontRightChassisAngularOffset);

  private final SwerveModule m_rearLeft =
      new SwerveModule(
          RobotConstants.kRearLeftDrivingCanId,
          RobotConstants.kRearLeftTurningCanId,
          RobotConstants.kBackLeftChassisAngularOffset);

  private final SwerveModule m_rearRight =
      new SwerveModule(
          RobotConstants.kRearRightDrivingCanId,
          RobotConstants.kRearRightTurningCanId,
          RobotConstants.kBackRightChassisAngularOffset);

  private final SendableChooser<String> m_chooser = new SendableChooser<>();
  // The gyro sensor
  private AHRS m_gyro = new AHRS(NavXComType.kUSB1);
  // Slew rate filter variables for controlling lateral acceleration
  private double m_currentRotation = 0.0;
  private double m_currentTranslationDir = 0.0;
  private double m_currentTranslationMag = 0.0;

  private double x;
  private double y;
  private double r;

  public static double maxSpeedCmd;

  private boolean waiting = false;
  private double maxSpeedDrive;
  private double maxSpeedTurn;

  private SlewRateLimiter m_magLimiter = new SlewRateLimiter(DriveConstants.kMagnitudeSlewRate);
  private SlewRateLimiter m_magLimiter1 = new SlewRateLimiter(DriveConstants.kMagnitudeSlewRate);
  private SlewRateLimiter m_rotLimiter = new SlewRateLimiter(DriveConstants.kRotationalSlewRate);
  private double m_prevTime = WPIUtilJNI.now() * 1e-6;

  /* Updating vision-based pose estimates might be expensive, so we can optionally
   ** update every VISION_UPDATE_INTERVAL periodic cycles instead. */
  private static int visionUpdateCounter = 0;
  private static final int VISION_UPDATE_INTERVAL = 10;

  // PoseEstimator class for tracking robot pose

  SwerveDrivePoseEstimator m_poseEstimator =
      new SwerveDrivePoseEstimator(
          DriveConstants.kDriveKinematics,
          Rotation2d.fromDegrees(getHeading()),
          new SwerveModulePosition[] {
            m_frontLeft.getPosition(),
            m_frontRight.getPosition(),
            m_rearLeft.getPosition(),
            m_rearRight.getPosition()
          },
          new Pose2d(),
          // Not tuned, maybe should be in constants, need to be tuned
          VecBuilder.fill(0.05, 0.05, Units.degreesToRadians(5)),
          VecBuilder.fill(0.5, 0.5, Units.degreesToRadians(30)));

  // Odometry class for tracking robot pose
  SwerveDriveOdometry m_odometry =
      new SwerveDriveOdometry(
          DriveConstants.kDriveKinematics,
          Rotation2d.fromDegrees(getHeading()),
          new SwerveModulePosition[] {
            m_frontLeft.getPosition(),
            m_frontRight.getPosition(),
            m_rearLeft.getPosition(),
            m_rearRight.getPosition()
          });

  SwerveDriveOdometry m_autoOdometry =
      new SwerveDriveOdometry(
          DriveConstants.kDriveKinematics,
          Rotation2d.fromDegrees(getHeading()),
          new SwerveModulePosition[] {
            m_frontLeft.getPosition(),
            m_frontRight.getPosition(),
            m_rearLeft.getPosition(),
            m_rearRight.getPosition()
          });

  /** Creates a new DriveSubsystem. */
  public DrivetrainSubsystem() {
    zeroHeading();
  }

  /**
   * Method to drive the robot using joystick info. (Field Oriented)
   *
   * @param xSpeed Speed of the robot in the x direction (forward).
   * @param ySpeed Speed of the robot in the y direction (sideways).
   * @param rot Angular rate of the robot.
   * @param maxTurn Max angular speed.
   * @param maxDrive Max driving speed.
   */
  public void fieldDrive(
      double xSpeed, double ySpeed, double rot, double maxTurn, double maxDrive) {
    maxSpeedDrive = maxDrive;
    maxSpeedTurn = maxTurn;

    double xSpeedCommanded;
    double ySpeedCommanded;

    xSpeedCommanded = m_magLimiter.calculate(xSpeed);
    ySpeedCommanded = m_magLimiter1.calculate(ySpeed);
    m_currentRotation = m_rotLimiter.calculate(rot);

    // Convert the commanded speeds into the correct units for the drivetrain
    double xSpeedDelivered = xSpeedCommanded * maxSpeedDrive;
    double ySpeedDelivered = ySpeedCommanded * maxSpeedDrive;
    double rotDelivered = m_currentRotation * maxSpeedTurn;

    x = xSpeedDelivered;
    y = ySpeedDelivered;
    r = rotDelivered;

    var swerveModuleStates =
        DriveConstants.kDriveKinematics.toSwerveModuleStates(
            ChassisSpeeds.fromFieldRelativeSpeeds(
                xSpeedDelivered, ySpeedDelivered, rotDelivered, getRotation2d()));

    SwerveDriveKinematics.desaturateWheelSpeeds(
        swerveModuleStates, DriveConstants.kMaxSpeedMetersPerSecond);
    m_frontLeft.setDesiredState(swerveModuleStates[0]);
    m_frontRight.setDesiredState(swerveModuleStates[1]);
    m_rearLeft.setDesiredState(swerveModuleStates[2]);
    m_rearRight.setDesiredState(swerveModuleStates[3]);
  }

  /**
   * Method to drive the robot using joystick info. (Robot Oriented)
   *
   * @param xSpeed Speed of the robot in the x direction (forward).
   * @param ySpeed Speed of the robot in the y direction (sideways).
   * @param rot Angular rate of the robot.
   * @param maxTurn Max angular speed.
   * @param maxDrive Max driving speed.
   */
  public void robotDrive(
      double xSpeed, double ySpeed, double rot, double maxTurn, double maxDrive) {
    maxSpeedDrive = maxDrive;
    maxSpeedTurn = maxTurn;

    double xSpeedCommanded;
    double ySpeedCommanded;

    xSpeedCommanded = m_magLimiter.calculate(xSpeed);
    ySpeedCommanded = m_magLimiter1.calculate(ySpeed);
    m_currentRotation = m_rotLimiter.calculate(rot);

    // Convert the commanded speeds into the correct units for the drivetrain
    double xSpeedDelivered = xSpeedCommanded * maxSpeedDrive;
    double ySpeedDelivered = ySpeedCommanded * maxSpeedDrive;
    double rotDelivered = m_currentRotation * maxSpeedTurn;

    x = xSpeedDelivered;
    y = ySpeedDelivered;
    r = rotDelivered;

    var swerveModuleStates =
        DriveConstants.kDriveKinematics.toSwerveModuleStates(
            new ChassisSpeeds(xSpeedDelivered, ySpeedDelivered, rotDelivered));

    SwerveDriveKinematics.desaturateWheelSpeeds(
        swerveModuleStates, DriveConstants.kMaxSpeedMetersPerSecond);
    m_frontLeft.setDesiredState(swerveModuleStates[0]);
    m_frontRight.setDesiredState(swerveModuleStates[1]);
    m_rearLeft.setDesiredState(swerveModuleStates[2]);
    m_rearRight.setDesiredState(swerveModuleStates[3]);
  }

  /** Sets the wheels into an X formation to prevent movement. */
  // Not used could be applied later though
  public void setX() {
    m_frontLeft.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
    m_frontRight.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));
    m_rearLeft.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));
    m_rearRight.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
  }

  /**
   * Sets the swerve ModuleStates.
   *
   * @param desiredStates The desired SwerveModule states.
   */
  public void setModuleStates(SwerveModuleState[] desiredStates) {
    SwerveDriveKinematics.desaturateWheelSpeeds(
        desiredStates, DriveConstants.kMaxSpeedMetersPerSecond);
    m_frontLeft.setDesiredState(desiredStates[0]);
    m_frontRight.setDesiredState(desiredStates[1]);
    m_rearLeft.setDesiredState(desiredStates[2]);
    m_rearRight.setDesiredState(desiredStates[3]);
  }

  /** Resets the drive encoders to currently read a position of 0. */
  public void resetEncoders() {
    m_frontLeft.resetEncoders();
    m_rearLeft.resetEncoders();
    m_frontRight.resetEncoders();
    m_rearRight.resetEncoders();
  }

  /** Zeroes the heading of the robot. */
  public void zeroHeading() {
    m_gyro.reset();
  }

  /** gives total yaw rotation */
  public double gyroangle() {
    return m_gyro.getAngle() * (RobotConstants.kGyroReversed ? -1.0 : 1.0);
  }

  public double getHeading() {
    return Math.IEEEremainder(m_gyro.getAngle() * (RobotConstants.kGyroReversed ? -1.0 : 1.0), 360);
  }

  public Rotation2d getRotation2d() {
    return Rotation2d.fromDegrees(getHeading());
  }

  /**
   * Returns the currently-estimated pose of the robot.
   *
   * @return The pose.
   */
  public Pose2d getPose() {
    return m_odometry.getPoseMeters();
  }

  /**
   * Resets the odometry to the specified pose.
   *
   * @param pose The pose to which to set the odometry.
   */
  public void resetOdometry(Pose2d pose) {
    m_odometry.resetPosition(
        Rotation2d.fromDegrees(getHeading()),
        new SwerveModulePosition[] {
          m_frontLeft.getPosition(),
          m_frontRight.getPosition(),
          m_rearLeft.getPosition(),
          m_rearRight.getPosition()
        },
        pose);
  }

  public void stopModules() {
    m_frontLeft.stop();
    m_frontRight.stop();
    m_rearLeft.stop();
    m_rearRight.stop();
  }

  public String getDropDown() {
    return m_chooser.getSelected();
  }

  public void setWait() {
    waiting = true;
  }

  public void setGo() {
    waiting = false;
  }

  /**
   * Returns the turn rate of the robot.
   *
   * @return The turn rate of the robot, in degrees per second
   */
  public double getTurnRate() {
    return m_gyro.getRate() * (RobotConstants.kGyroReversed ? -1.0 : 1.0);
  }

  /**
   * Updates the field relative position of the robot. From
   * https://github.com/LimelightVision/limelight-examples/blob/main/java-wpilib/swerve-megatag-odometry/src/main/java/frc/robot/Drivetrain.java
   *
   * <p>Megatag 1 code removed and odometry object renamed.
   */
  public void updateOdometry() {
    m_odometry.update(
        Rotation2d.fromDegrees(getHeading()),
        new SwerveModulePosition[] {
          m_frontLeft.getPosition(),
          m_frontRight.getPosition(),
          m_rearLeft.getPosition(),
          m_rearRight.getPosition()
        });
  }

  public void updatePoseEstimate(boolean updateVision) {
    m_poseEstimator.update(
        Rotation2d.fromDegrees(getHeading()),
        new SwerveModulePosition[] {
          m_frontLeft.getPosition(),
          m_frontRight.getPosition(),
          m_rearLeft.getPosition(),
          m_rearRight.getPosition()
        });

    if (!updateVision) return;
    boolean doRejectUpdate = false;

    LimelightHelpers.SetRobotOrientation(
        "limelight",
        m_poseEstimator.getEstimatedPosition().getRotation().getDegrees(),
        0,
        0,
        0,
        0,
        0);
    LimelightHelpers.PoseEstimate mt2 =
        LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2("limelight");
    if (Math.abs(m_gyro.getRate())
        > 720) // if our angular velocity is greater than 720 degrees per second, ignore vision
    // updates
    {
      doRejectUpdate = true;
    }
    // If there's no Limelight attached, mt2 will be null.
    if (mt2 == null || mt2.tagCount == 0) {
      doRejectUpdate = true;
    }
    if (!doRejectUpdate) {
      // TODO: tune these constants or adjust based on distance?
      // m_odometry.setVisionMeasurementStdDevs(VecBuilder.fill(5, 5, 500));
      m_poseEstimator.addVisionMeasurement(mt2.pose, mt2.timestampSeconds);
    }
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Z axis angle", getHeading());
    SmartDashboard.putBoolean("Auto is Waiting", waiting);
    SmartDashboard.putNumber("controller speed", maxSpeedCmd);

    SmartDashboard.putNumber("Gyro Angle", getHeading());

    updateOdometry();

    updatePoseEstimate(visionUpdateCounter == 0);
    visionUpdateCounter = (visionUpdateCounter + 1) % VISION_UPDATE_INTERVAL;
  }
}
