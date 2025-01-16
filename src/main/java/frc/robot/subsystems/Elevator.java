// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Rotation;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ElevatorConstants;

public class Elevator extends SubsystemBase {
  /** Creates a new Elevator. */

  private enum ControlMode {
    kOpenLoop, kPID
  }

  private ControlMode m_controlMode = ControlMode.kOpenLoop;
  private final PIDController m_pidController = new PIDController(0, 0, 0);
  private double m_demand;
  private final TalonFX m_motor;
  private final TalonFX m_followerMotor;
  double outputVoltage = 0;

  public Elevator() {

    TalonFXConfiguration config = new TalonFXConfiguration();

    config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    config.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
    config.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
    config.SoftwareLimitSwitch.ReverseSoftLimitThreshold = 5;
    config.SoftwareLimitSwitch.ForwardSoftLimitThreshold = 80;
    config.CurrentLimits.SupplyCurrentLimitEnable = true;
    config.CurrentLimits.SupplyCurrentLimit = 40;
    config.CurrentLimits.StatorCurrentLimitEnable = false;
    config.CurrentLimits.StatorCurrentLimit = 125;
    config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

    m_motor = new TalonFX(ElevatorConstants.kMotorID);
    m_motor.getConfigurator().apply(config);
    m_motor.getVelocity().setUpdateFrequency(50);
    m_motor.optimizeBusUtilization();

    m_followerMotor = new TalonFX(ElevatorConstants.kFollowerMotorID);
    m_followerMotor.optimizeBusUtilization();
    m_followerMotor.setControl(new Follower(ElevatorConstants.kMotorID, true));

    SmartDashboard.putNumber("Elevator kP", ElevatorConstants.kP);
    SmartDashboard.putNumber("Elevator kI", ElevatorConstants.kI);
    SmartDashboard.putNumber("Elevator kD", ElevatorConstants.kD);
    SmartDashboard.putNumber("Elevator kF", ElevatorConstants.kF);
  }

  public double getPosition() {
    return m_motor.getPosition().getValueAsDouble() * ElevatorConstants.kRotationToInches;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    switch (m_controlMode) {

      case kOpenLoop:
        // Do openloop stuff here

        outputVoltage = m_demand;

        break;

      case kPID:

        m_pidController.setP(SmartDashboard.getNumber("Elevator kP", 0));
        m_pidController.setI(SmartDashboard.getNumber("Elevator kI", 0));
        m_pidController.setD(SmartDashboard.getNumber("Elevator kD", 0));
        double kF = SmartDashboard.getNumber("Elevator kF", 0);

        outputVoltage = kF * m_demand + m_pidController.calculate(getPosition(), m_demand);

        break;
      default:
        // What happened!?
        break;
    }

    // Publish to smart dashboard
    // // SmartDashboard.putNumber("elevator " + (m_isInverted ? "Left" : "Right")+"
    // // Velocity", getVelocity());
    // SmartDashboard.putNumber("elevator " + (m_isInverted ? "Left" : "Right") + "
    // output voltage", outputVoltage);
    // // SmartDashboard.putNumber("elevator "+ (m_isInverted ? "Left" : "Right") +
    // "
    // // RPM Error", getErrorRPM());
    // // SmartDashboard.putNumber("elevator " + (m_isInverted ? "Left" : "Right") +
    // "
    // // RPM Percent Error", getErrorPercent());
    // SmartDashboard.putString("elevator " + (m_isInverted ? "Left" : "Right") + "
    // Control Mode",
    // m_controlMode.toString());
    // // SmartDashboard.putBoolean("elevator InRange"+ (m_isInverted ? "Left" :
    // // "Right"), isErrorInRange());
    // SmartDashboard.putNumber("elevator " + (m_isInverted ? "Left" : "Right") +
    // "elevator rotation",
    // m_motor.getPosition().getValueAsDouble());
    SmartDashboard.putNumber("Height", getPosition());
    SmartDashboard.putNumber("demand", m_demand);

    if (m_motor.getPosition().getValueAsDouble() < 5 && outputVoltage < 0) {
      outputVoltage = 0;
    } else if (m_motor.getPosition().getValueAsDouble() > 80 && outputVoltage > 0) {
      outputVoltage = 0;
    } else {
      m_motor.setVoltage(outputVoltage);
      m_followerMotor.setVoltage(outputVoltage);
    }
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
    return openLoopCommand(() -> OutputVoltage);
  }

  public void stop() {
    setOutputVoltage(0);
  }

  // pid
  public double getErrorRotation() {
    if (m_controlMode == ControlMode.kPID) {
      return m_pidController.getPositionError();
    }
    return 0;
  }

  public boolean isErrorInRange() {
    return (-4 < this.getErrorPercent() && this.getErrorPercent() < 4);
  }

  public double getErrorPercent() {
    if (m_controlMode == ControlMode.kPID) {
      return (m_demand - getPosition()) / m_demand * 10;
    }
    return 0;
  }
  
  public Command waitUntilErrorInrange() {
    return Commands.waitUntil(() -> this.isErrorInRange());
  }

  public boolean isErrorOutOfRange() {
    return (this.getErrorPercent() > 15);
  }

  public Command waitUntilErrorOutOfRange() {
    return Commands.waitUntil(() -> this.isErrorOutOfRange());

  }

  public void setPIDSetpoint(double inches) {
    m_controlMode = ControlMode.kPID;
    m_demand = inches;
  }

  public Command pidCommand(DoubleSupplier rpmSupplier) {
    return Commands.runEnd(
        () -> this.setPIDSetpoint(rpmSupplier.getAsDouble()), this::stop, this);
  }

  public Command pidCommand(double inches) {
    return pidCommand(() -> inches);
  }

}
