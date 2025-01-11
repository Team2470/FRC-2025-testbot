// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.reduxrobotics.sensors.canandcolor.Canandcolor;
import com.reduxrobotics.sensors.canandcolor.CanandcolorSettings;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.reduxrobotics.canand.CanandEventLoop;

public class ReduxColorSensor extends SubsystemBase {
  /** Creates a new ReduxColorSensor. */
  Canandcolor canandcolor = new Canandcolor(0);
  CanandcolorSettings settings = new CanandcolorSettings();

  public ReduxColorSensor() {
    CanandEventLoop.getInstance();
    settings.setLampLEDBrightness(0.0);

  }

  @Override
  public void periodic() {
    SmartDashboard.putBoolean("Color sensor connected",canandcolor.isConnected());
    SmartDashboard.putNumber("Color sensor temperature", canandcolor.getTemperature());
    // This method will be called once per scheduler run
  }
}
