// The subsystem handles power-related diagnostics, as well
// as identifying the current battery in the logs.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class PowerSubsystem extends SubsystemBase {

  // Battery ID for logging
  private final SendableChooser<String> battery_chooser = new SendableChooser<>();
  private String batteryID;

  public PowerSubsystem() {

    // Enumerate all batteries here with a unique name. When this option changes,
    // we'll log this so we can connect log diagnostics with specific batteries.
    battery_chooser.setDefaultOption("Unknown", "Unknown Battery");
    battery_chooser.addOption("2021-2", "2021-2");
    battery_chooser.addOption("2021-3", "2021-3");
    battery_chooser.addOption("2021-4", "2021-4");
    battery_chooser.addOption("2021-5", "2021-5");
    battery_chooser.addOption("2021-X", "2021-X");
    battery_chooser.addOption("2022-1", "2022-1");
    battery_chooser.addOption("2022-2", "2022-2");
    battery_chooser.addOption("grey_1", "grey_1");
    battery_chooser.addOption("2024-1", "2024-1");
    battery_chooser.addOption("2024-2", "2024-2");
    battery_chooser.addOption("2024-3", "2024-3");
    battery_chooser.addOption("2025-1", "2025-1");
    battery_chooser.addOption("2025-2", "2025-2");
    SmartDashboard.putData("Battery Selector", battery_chooser);
  }

  public void periodic() {

    String oldBatteryID = batteryID;
    batteryID = battery_chooser.getSelected();
    if (batteryID != null && !batteryID.equals(oldBatteryID)) {
      DataLogManager.log("Battery selected: " + batteryID);
    }

    // TODO: add more power-related logging here?
  }

  public String getBatteryID() {
    return batteryID;
  }
}
