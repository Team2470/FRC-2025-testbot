package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.controls.*;
import com.ctre.phoenix6.configs.*;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.jni.PlatformJNI;
import com.ctre.phoenix6.sim.DeviceType;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.*;
import com.ctre.phoenix6.spns.*;
import com.ctre.phoenix6.signals.*;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.units.*;
import edu.wpi.first.units.measure.*;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.ctre.phoenix6.hardware.TalonFX;

public class Wrist extends SubsystemBase {
    private enum ControlMode {
        kOpenLoop, kPID, kStop, kHoming
    }
    //
    // Hardware
    //
    private final TalonFX m_motor;

    //
    // State
    //
    private ControlMode m_controlMode = ControlMode.kStop;
    private double m_demand;
    
    public Wrist (int motorid) {
        m_motor = new TalonFX(motorid, "Canivore");
        TalonFXConfiguration config = new TalonFXConfiguration();
    }
    @Override
    public void periodic () {
        double outputVoltage = 6;
        switch (m_controlMode) {
            case kStop:
      
              outputVoltage = 0;
              break;
      
            case kOpenLoop:
              // Do openloop stuff here
      
              outputVoltage = m_demand;
      
              break;
        }

        m_motor.setVoltage(outputVoltage);

    }
    public void setOutputVoltage(double OutputVoltage) {
        m_controlMode = ControlMode.kOpenLoop;
        m_demand = OutputVoltage;
    }

    public Command openLoopCommand(DoubleSupplier OutputVoltageSupplier) {
        return Commands.runEnd(
            () -> this.setOutputVoltage(OutputVoltageSupplier.getAsDouble()), this::stop, this);

    }

    public Command openLoopCommand(double OutputVoltage) {
        return openLoopCommand( () -> OutputVoltage);
    }

    public void stop() {
        m_controlMode = ControlMode.kStop;
        m_demand = 0;
    }

}
