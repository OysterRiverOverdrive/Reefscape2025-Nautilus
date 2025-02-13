// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.DriveConstants;
import frc.robot.auto.*;
// import frc.robot.auto.plans.*;
import frc.robot.commands.TeleopCmd;
import frc.robot.commands.algaeArm.*;
import frc.robot.commands.coralIntake.*;
import frc.robot.commands.elevator.*;
import frc.robot.subsystems.AlgaeArmSubsystem;
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
  private final ElevatorSubsystem elevator = new ElevatorSubsystem();
  private final CoralIntakeSubsystem coralIntake = new CoralIntakeSubsystem();
  private final AlgaeArmSubsystem algaeArm = new AlgaeArmSubsystem();
  private final PowerSubsystem battery = new PowerSubsystem();

  // Commands
  private final TeleopCmd teleopCmd =
      new TeleopCmd(
          drivetrain,
          () -> cutil.Boolsupplier(Controllers.ps4_LB, DriveConstants.joysticks.DRIVER));

  public RobotContainer() {

    DataLogManager.start();

    // Declare default command during Teleop Period as TeleopCmd(Driving Command)
    drivetrain.setDefaultCommand(teleopCmd);

    // Add Auto options to dropdown and push to dashboard
    m_chooser.setDefaultOption("Auto[Rename Me]", auto1);
    m_chooser.addOption("Auto[Rename Me]", auto2);
    m_chooser.addOption("Auto[Rename Me]", auto3);
    m_chooser.addOption("Auto[Rename Me]", auto4);
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

    // Algae Spinner Bindings
    cutil
        .supplier(Controllers.xbox_rb, DriveConstants.joysticks.OPERATOR)
        .onTrue(new AlgaeSpinnerForwardCommand(algaeArm))
        .onFalse(
            cutil.getTriggerButton(Controllers.xbox_lt, 0.2, DriveConstants.joysticks.OPERATOR)
                ? new AlgaeSpinnerReverseCommand(algaeArm)
                : new AlgaeSpinnerStopCommand(algaeArm));
    cutil
        .triggerSupplier(Controllers.xbox_rt, 0.2, DriveConstants.joysticks.OPERATOR)
        .onTrue(new AlgaeSpinnerReverseCommand(algaeArm))
        .onFalse(
            cutil.Boolsupplier(Controllers.xbox_lb, DriveConstants.joysticks.OPERATOR)
                ? new AlgaeSpinnerForwardCommand(algaeArm)
                : new AlgaeSpinnerStopCommand(algaeArm));

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
        .supplier(Controllers.xbox_options, DriveConstants.joysticks.OPERATOR)
        .onTrue(new InstantCommand(() -> elevator.toBase()));

    // Intake sequential command binding
    cutil
        .supplier(Controllers.xbox_a, DriveConstants.joysticks.OPERATOR)
        .onTrue(
            new SequentialCommandGroup(
                new ElevatorToAboveIntakeCommand(elevator),
                new AlgaeArmToUpCommand(algaeArm),
                new ElevatorToIntakeCommand(elevator),
                new CoralIntakeForwardCommand(coralIntake)))
        .onFalse(
            new SequentialCommandGroup(
                new CoralIntakeStopCommand(coralIntake),
                new ElevatorToAboveIntakeCommand(elevator),
                new AlgaeArmToDownCommand(algaeArm)));

    // Coral Intake Bindings
    cutil
        .supplier(Controllers.xbox_lb, DriveConstants.joysticks.OPERATOR)
        .onTrue(new CoralIntakeForwardCommand(coralIntake))
        .onFalse(
            cutil.getTriggerButton(Controllers.xbox_lt, 0.2, DriveConstants.joysticks.OPERATOR)
                ? new CoralIntakeReverseCommand(coralIntake)
                : new CoralIntakeStopCommand(coralIntake));
    cutil
        .triggerSupplier(Controllers.xbox_lt, 0.2, DriveConstants.joysticks.OPERATOR)
        .onTrue(new CoralIntakeReverseCommand(coralIntake))
        .onFalse(
            cutil.Boolsupplier(Controllers.xbox_lb, DriveConstants.joysticks.OPERATOR)
                ? new CoralIntakeForwardCommand(coralIntake)
                : new CoralIntakeStopCommand(coralIntake));

    // Algae Arm Bindings
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
        break;
      case auto2:
        break;
      case auto3:
        break;
      case auto4:
        break;
      case auto5:
        break;
      case auto6:
        break;
      case auto7:
        break;
    }
    // Create sequential command with the wait command first then run selected auto
    auto =
        new SequentialCommandGroup(
            new BeginSleepCmd(drivetrain, SmartDashboard.getNumber("Auto Wait Time (Sec)", 0)),
            auto);
    return auto;
  }
}
