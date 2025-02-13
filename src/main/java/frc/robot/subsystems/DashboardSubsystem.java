// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.TimerConstants;

public final class DashboardSubsystem {
  // determinew what timer is enabled
  static boolean TeleOpStartTimerEnabled = false;
  static boolean TeleOpEndTimerEnabled = false;
  static boolean AutoTimerEnabled = false;

  // Is the the ammount of times RobotPeriodic in robot.java has passed (updates every 20ms)
  static float TeleOpEndTimerValue = 0;
  static float TeleOpStartTimerValue = 0;
  static float AutoTimerValue = 0;
  // check Constants for timervalues
  private SendableChooser<Double> m_allianceChoice = new SendableChooser<Double>();
  private Double blue = -1.0;
  private Double red = 1.0;

  public DashboardSubsystem() {
    m_allianceChoice.setDefaultOption("Red Alliance", red);
    m_allianceChoice.addOption("Blue Alliance", blue);
    SmartDashboard.putData("Alliance Override", m_allianceChoice);
  }

  public double getAlliance() {
    return m_allianceChoice.getSelected();
  }

  public static void StartTimer(String type, float timer_length) {

    switch (type) {
      case "TeleOpStartTimer":
        TeleOpStartTimerEnabled = true;
        SmartDashboard.putNumber("TeleOpStartTimer", timer_length);
        TeleOpStartTimerValue = timer_length;
        break;
      case "TeleOpEndTimer":
        TeleOpEndTimerEnabled = true;
        SmartDashboard.putNumber("TeleOpEndTimer", timer_length);
        TeleOpEndTimerValue = timer_length;
        break;
      case "AutoTimer":
        AutoTimerEnabled = true;
        SmartDashboard.putNumber("AutoTimer", timer_length);
        AutoTimerValue = timer_length;
        break;
    }
  }

  public static void RunTimers() {
    // will run code if 1 second has passed
    if (TeleOpStartTimerEnabled) {
      if (SmartDashboard.getNumber("TeleOpStartTimer", 0) <= 0) {
        SmartDashboard.putNumber("TeleOpStartTimer", 0);

        TeleOpStartTimerEnabled = false;
        // will start second timer after this ends
        StartTimer("TeleOpEndTimer", TimerConstants.TeleOpEndTimerLength);

      } else {
        TeleOpStartTimerValue = FormatNumber((float) (TeleOpStartTimerValue - .02f));

        SmartDashboard.putNumber("TeleOpStartTimer", TeleOpStartTimerValue);
      }
    }
    if (TeleOpEndTimerEnabled) {
      if (SmartDashboard.getNumber("TeleOpEndTimer", 0) <= 0) {
        TeleOpEndTimerEnabled = false;

      } else {
        TeleOpEndTimerValue = FormatNumber((float) (TeleOpEndTimerValue - .02f));
        SmartDashboard.putNumber("TeleOpEndTimer", TeleOpEndTimerValue);
      }
    }
    if (AutoTimerEnabled) {
      if (SmartDashboard.getNumber("AutoTimer", 0) <= 0) {
        AutoTimerEnabled = false;

      } else {
        AutoTimerValue = FormatNumber((float) (AutoTimerValue - .02f));
        SmartDashboard.putNumber("AutoTimer", AutoTimerValue);
      }
    }
  }

  public static float FormatNumber(float broken) {
    return (Math.round(broken * 1000f) / 1000f);
  }

  // check enabled if enabled
  public static boolean isTeleOpStartTimerEnabled() {
    return TeleOpStartTimerEnabled;
  }

  public static boolean isTeleOpEndTimerEnabled() {
    return TeleOpEndTimerEnabled;
  }

  public static boolean isAutoTimerEnabled() {
    return AutoTimerEnabled;
  }

  // get timer values
  public static float getTeleOpStartTimerValue() {
    return (float) SmartDashboard.getNumber("TeleOpTimerStart", 0);
  }

  public static float getTeleOpEndTimerValue() {
    return (float) SmartDashboard.getNumber("TeleOpTimerEnd", 0);
  }

  public static float getAutoTimerValue() {
    return (float) SmartDashboard.getNumber("AutoTimer", 0);
  }

  // set status timers status
  public static void setAutoTimerEnabled() {
    AutoTimerEnabled = true;
  }

  public static void setTeleOpStartTimerEnabled() {
    TeleOpStartTimerEnabled = true;
  }

  public static void setTeleOpEndTimerEnabled() {
    TeleOpEndTimerEnabled = true;
  }

  public static void setAutoTimerDisabled() {
    AutoTimerEnabled = false;
  }

  public static void setTeleOpStartTimerDisabled() {
    TeleOpStartTimerEnabled = false;
  }

  public static void setTeleOpEndTimerDisabled() {
    TeleOpEndTimerEnabled = false;
  }
}
