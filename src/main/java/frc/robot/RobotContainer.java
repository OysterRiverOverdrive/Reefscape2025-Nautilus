// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Constants.DriveConstants;
import frc.robot.auto.*;
import frc.robot.auto.plans.*;
import frc.robot.commands.TeleopCmd;
import frc.robot.commands.coralIntake.*;
import frc.robot.commands.elevator.*;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.CoralIntakeSubsystem;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.PowerSubsystem;
import frc.utils.ControllerUtils;

public class RobotContainer {
  // Controller Utils Instance
  private final ControllerUtils cutil = new ControllerUtils();

  // Auto Dropdown - Make dropdown variable and variables to be selected
  private final SendableChooser<String> m_chooser = new SendableChooser<>();
  private final String rightThree = "1";
  private final String leftThree = "2";
  private final String rightOne = "3";
  private final String leftOne = "4";
  private final String middleOne = "5";
  private final String auto6 = "6";
  private final String auto7 = "7";
  Command auto;

  // Subsystems
  private final DrivetrainSubsystem drivetrain = new DrivetrainSubsystem();
  private final LimelightSubsystem limelight = new LimelightSubsystem(drivetrain);
  private final ElevatorSubsystem elevator = new ElevatorSubsystem(drivetrain);
  private final CoralIntakeSubsystem coralIntake = new CoralIntakeSubsystem();
  private final PowerSubsystem battery = new PowerSubsystem();
  private final ClimberSubsystem climber = new ClimberSubsystem();

  // Commands
  private final TeleopCmd teleopCmd =
      new TeleopCmd(
          drivetrain,
          () -> cutil.Boolsupplier(Controllers.xbox_lb, DriveConstants.joysticks.DRIVER));
  private final ElevTPIDCmd elevTPIDCmd = new ElevTPIDCmd(elevator);

  // AUTOS
  private final ThreeCoralRight rightThreeCoralPlan =
      new ThreeCoralRight(drivetrain, elevator, coralIntake);
  private final RightOneCoralPlan rightOneCoralPlan =
      new RightOneCoralPlan(drivetrain, elevator, coralIntake);
  private final ThreeCoralLeft leftThreeCoralPlan =
      new ThreeCoralLeft(drivetrain, elevator, coralIntake);
  private final MiddleOneCoralPlan middleOneCoralPlan =
      new MiddleOneCoralPlan(drivetrain, elevator, coralIntake);
  private final LeftOneCoralPlan leftOneCoralPlan =
      new LeftOneCoralPlan(drivetrain, elevator, coralIntake);

  public RobotContainer() {

    DataLogManager.start();

    // Default Commands to be run all the time, only one per subsystem
    drivetrain.setDefaultCommand(teleopCmd);
    elevator.setDefaultCommand(elevTPIDCmd);
    // coralIntake.setDefaultCommand(new CoralIntakeStopCommand(coralIntake));

    // Add Auto options to dropdown and push to dashboard
    m_chooser.setDefaultOption("RightThreeCoralPlan", rightThree);
    m_chooser.addOption("RightOneCoralPlan", rightOne);
    m_chooser.addOption("LeftThreeCoralPlan", leftThree);
    m_chooser.addOption("MiddleOneCoralPlan", middleOne);
    m_chooser.addOption("LeftOneCoralPlan", leftOne);
    m_chooser.addOption("Auto[Rename Me]", auto6);
    m_chooser.addOption("Auto[Rename Me]", auto7);
    SmartDashboard.putData("Auto Selector", m_chooser);
    SmartDashboard.putNumber("Auto Wait Time (Sec)", 0);

    // Configure Buttons Methods
    configureBindings();
  }

  // Continuation of method to prevent double instanciation
  public void toBase() {
    elevator.toBase();
  }

  private void configureBindings() {
    // Configure buttons
    // Prior Reference:
    // https://github.com/OysterRiverOverdrive/Charged-Up-2023-Atlas_Chainsaw/blob/main/src/main/java/frc/robot/RobotContainer.java
    cutil
        .supplier(Controllers.ps4_RB, DriveConstants.joysticks.DRIVER)
        .onTrue(new InstantCommand(() -> drivetrain.zeroHeading()));

    // Elevator Bindings
    cutil.POVsupplier(0, DriveConstants.joysticks.OPERATOR)
        .onTrue(new InstantCommand(() -> elevator.toL1()));
    cutil.POVsupplier(90, DriveConstants.joysticks.OPERATOR)
        .onTrue(new InstantCommand(() -> elevator.toL2()));
    cutil.POVsupplier(180, DriveConstants.joysticks.OPERATOR)
        .onTrue(new InstantCommand(() -> elevator.toL3()));
    cutil.POVsupplier(270, DriveConstants.joysticks.OPERATOR)
        .onTrue(new InstantCommand(() -> elevator.toL4()));
    cutil
        .supplier(Controllers.xbox_share, DriveConstants.joysticks.OPERATOR)
        .onTrue(new InstantCommand(() -> elevator.toBase()));
    cutil
        .triggerSupplier(Controllers.xbox_lt, 0.2, DriveConstants.joysticks.OPERATOR)
        .onTrue(new InstantCommand(() -> elevator.toIntake()));

    // Coral Intake Bindings
    cutil
        .supplier(Controllers.xbox_rb, DriveConstants.joysticks.OPERATOR)
        .onTrue(new CoralIntakeForwardCommand(coralIntake))
        .onFalse(new CoralIntakeStopCommand(coralIntake));
    cutil
        .triggerSupplier(Controllers.xbox_rt, 0.2, DriveConstants.joysticks.OPERATOR)
        .onTrue(new CoralIntakeReverseCommand(coralIntake))
        .onFalse(new CoralIntakeStopCommand(coralIntake));

    // Climber Bindings
    cutil
        .triggerSupplier(Controllers.xbox_rt, 0.2, DriveConstants.joysticks.DRIVER)
        .onTrue(new InstantCommand(() -> climber.climberForwardCmd()))
        .onFalse(new InstantCommand(() -> climber.climberStopCmd()));
    cutil
        .triggerSupplier(Controllers.xbox_lt, 0.2, DriveConstants.joysticks.DRIVER)
        .onTrue(new InstantCommand(() -> climber.climberReverseCmd()))
        .onFalse(new InstantCommand(() -> climber.climberStopCmd()));
  }

  public Command getAutonomousCommand() {
    // Prior Reference:
    // https://github.com/OysterRiverOverdrive/Charged-Up-2023-Atlas_Chainsaw/blob/main/src/main/java/frc/robot/RobotContainer.java

    switch (m_chooser.getSelected()) {
      default:
      case rightThree:
        auto = rightThreeCoralPlan;
        break;
      case leftThree:
        auto = leftThreeCoralPlan;
        break;
      case rightOne:
        auto = rightOneCoralPlan;
        break;
      case leftOne:
        auto = leftOneCoralPlan;
        break;
      case middleOne:
        auto = middleOneCoralPlan;
        break;
      case auto6:
        break;
      case auto7:
        break;
    }
    return auto;
  }
}
