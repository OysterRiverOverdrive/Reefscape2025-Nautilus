// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Constants.DriveConstants;
import frc.robot.auto.*;
import frc.robot.auto.plans.*;
import frc.robot.commands.TeleopCmd;
import frc.robot.subsystems.AlgaeArmSubsystem;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.LimelightSubsystem;
import frc.utils.ControllerUtils;

public class RobotContainer {
  // Controller Utils Instance
  private final ControllerUtils cutil = new ControllerUtils();

  // Auto Dropdown - Make dropdown variable and variables to be selected
  private final SendableChooser<String> m_chooser = new SendableChooser<>();
  private final String auto1 = "1";
  private final String auto2 = "2";
  private final String auto3 = "3";
  private final String auto4 = "4";
  private final String auto5 = "5";
  private final String auto6 = "6";
  private final String auto7 = "7";
  Command auto;

  // Subsystems
  private final DrivetrainSubsystem drivetrain = new DrivetrainSubsystem();
  private final LimelightSubsystem limelight = new LimelightSubsystem();
  private final AlgaeArmSubsystem algaeArm = new AlgaeArmSubsystem();

  // Commands
  private final TeleopCmd teleopCmd =
      new TeleopCmd(
          drivetrain,
          () -> cutil.Boolsupplier(Controllers.ps4_LB, DriveConstants.joysticks.DRIVER));

  // AUTOS
  private final CoolTwoCoralAutoPlan coolTwoCOrealAuto = new CoolTwoCoralAutoPlan(drivetrain);
  private final LeftOneCoralPlan leftOneCoralPlan = new LeftOneCoralPlan(drivetrain);
  private final LeftThreeCoralPlan leftThreeCoralPlan = new LeftThreeCoralPlan(drivetrain);
  private final MiddleOneCoralPlan middleOneCoralPlan = new MiddleOneCoralPlan(drivetrain);

  public RobotContainer() {
    // Declare default command during Teleop Period as TeleopCmd(Driving Command)
    drivetrain.setDefaultCommand(teleopCmd);

    // Add Auto options to dropdown and push to dashboard
    m_chooser.setDefaultOption("CoolTwoCoralAutoPlanYay!!", auto1);
    m_chooser.addOption("LeftOneCoralPlan", auto2);
    m_chooser.addOption("LeftThreeCoralPlan", auto3);
    m_chooser.addOption("MiddleOneCoralPlan", auto4);
    m_chooser.addOption("Auto[Rename Me]", auto5);
    m_chooser.addOption("Auto[Rename Me]", auto6);
    m_chooser.addOption("Auto[Rename Me]", auto7);
    SmartDashboard.putData("Auto Selector", m_chooser);
    SmartDashboard.putNumber("Auto Wait Time (Sec)", 0);

    // Configure Buttons Methods
    configureBindings();
  }

  private void configureBindings() {
    // Configure buttons
    // Prior Reference:
    // https://github.com/OysterRiverOverdrive/Charged-Up-2023-Atlas_Chainsaw/blob/main/src/main/java/frc/robot/RobotContainer.java
    cutil
        .supplier(Controllers.ps4_RB, DriveConstants.joysticks.DRIVER)
        .onTrue(new InstantCommand(() -> drivetrain.zeroHeading()));
    cutil
        .supplier(Controllers.xbox_b, DriveConstants.joysticks.OPERATOR)
        .onTrue(new InstantCommand(() -> algaeArm.toDown()));
    cutil
        .supplier(Controllers.xbox_x, DriveConstants.joysticks.OPERATOR)
        .onTrue(new InstantCommand(() -> algaeArm.toFlat()));
    cutil
        .supplier(Controllers.xbox_y, DriveConstants.joysticks.OPERATOR)
        .onTrue(new InstantCommand(() -> algaeArm.toRemoveAlgae()));
  }

  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    // Return NOTHING, replace with command to be run in autonomous period
    // Prior Reference:
    // https://github.com/OysterRiverOverdrive/Charged-Up-2023-Atlas_Chainsaw/blob/main/src/main/java/frc/robot/RobotContainer.java

    switch (m_chooser.getSelected()) {
      default:
      case auto1:
        auto = coolTwoCOrealAuto;
        break;
      case auto2:
        auto = leftOneCoralPlan;
        break;
      case auto3:
        auto = leftThreeCoralPlan;
        break;
      case auto4:
        auto = middleOneCoralPlan;
        break;
      case auto5:
        break;
      case auto6:
        break;
      case auto7:
        break;
    }

    return auto;
  }
}
