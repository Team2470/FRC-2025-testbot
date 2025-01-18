// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ElevatorConstants;

public class Elevator extends SubsystemBase {
  /** Creates a new Elevator. */

  private enum ControlMode {
    kOpenLoop, kPID, kStop, kHoming
  }
  //
  // Hardware
  //
  private final TalonFX m_motor;
  private final TalonFX m_motorFollower;
  private final DigitalInput m_retractLimit;

  //
  //State
  //
  private ControlMode m_controlMode = ControlMode.kStop;
  private final PIDController m_pidController = new PIDController(0, 0, 0);
  private double m_demand;
  private boolean m_isHomed;
  


  public Elevator() {

    TalonFXConfiguration config = new TalonFXConfiguration();

    config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    config.MotorOutput.NeutralMode = NeutralModeValue.Brake;

    config.CurrentLimits.SupplyCurrentLimitEnable = true;
    config.CurrentLimits.SupplyCurrentLimit = 40;
    config.CurrentLimits.StatorCurrentLimitEnable = false;
    config.CurrentLimits.StatorCurrentLimit = 125;

    //
    // Setup Follower motor
    //
    m_motorFollower = new TalonFX(ElevatorConstants.kMotorFollowerID);
    m_motorFollower.getConfigurator().apply(config);
    m_motorFollower.optimizeBusUtilization();
    m_motorFollower.setControl(new Follower(ElevatorConstants.kMotorID, true));
    

    //
    // Apply add extra leader configuration on top of the base config
    // - Any control related settings for PID, Motion Magic, Remote Sensors, etc..
    // - Soft limits should only be applied to the leader motor, if they are applied
    // to the follower
    // it may stop moving if their built in encoders are not in sync.
    //
    config.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
    config.SoftwareLimitSwitch.ForwardSoftLimitThreshold = 80;
    config.SoftwareLimitSwitch.ReverseSoftLimitEnable = false;
    config.SoftwareLimitSwitch.ReverseSoftLimitThreshold = 0.25;

    m_motor = new TalonFX(ElevatorConstants.kMotorID);
    m_motor.getConfigurator().apply(config);

    // We want to read position data from the leader motor
    m_motor.getPosition().setUpdateFrequency(50);

    // These 3 are needed for the follower motor to work
    m_motor.getDutyCycle().setUpdateFrequency(50);
    m_motor.getMotorVoltage().setUpdateFrequency(50);
    m_motor.getTorqueCurrent().setUpdateFrequency(50);
    m_motor.optimizeBusUtilization();

    // Limit switch setup
    m_retractLimit = new DigitalInput(ElevatorConstants.kRetractLimitChannel);

    SmartDashboard.putNumber("Elevator kP", ElevatorConstants.kP);
    SmartDashboard.putNumber("Elevator kI", ElevatorConstants.kI);
    SmartDashboard.putNumber("Elevator kD", ElevatorConstants.kD);
    SmartDashboard.putNumber("Elevator kF", ElevatorConstants.kF);
  }

  public double getPosition() {
    return m_motor.getPosition().getValueAsDouble() * ElevatorConstants.kRotationToInches;
  }

  public boolean isRetracted() {
    return !m_retractLimit.get();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    double outputVoltage = 0;

    switch (m_controlMode) {

      case kHoming:
        // Do homing stuff here
        if (isRetracted() ) {
          m_motor.setVoltage(0);
          m_motor.setPosition(0);
          m_isHomed = true;
          m_controlMode = ControlMode.kStop;
        } else {
          outputVoltage = -2;
        }
        break;
      case kStop:
        outputVoltage = 0;
        break;

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

    SmartDashboard.putNumber("Elevator Height", getPosition());
    SmartDashboard.putNumber("Elevator Demand", m_demand);
    SmartDashboard.putNumber("Elevator HeightInRotations", m_motor.getPosition().getValueAsDouble());
    SmartDashboard.putBoolean("Elevator is Retracted", isRetracted());
    SmartDashboard.putBoolean("Elevator is Homed", m_isHomed);
    SmartDashboard.putString("Elevator Controlmode", m_controlMode.toString());
    SmartDashboard.putNumber("Elevator Demand", m_demand);

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
    return openLoopCommand(() -> OutputVoltage);
  }

  public Command elevatorHomeCommand() {
    return Commands.runEnd(() -> this.startHoming(), this::stop, this);
  }

  public void startHoming() {
    m_controlMode = ControlMode.kHoming;
  }

  public void stop() {
    m_controlMode = ControlMode.kStop;
    m_demand = 0;
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
