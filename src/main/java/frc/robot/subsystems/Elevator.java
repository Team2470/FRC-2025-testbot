// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Rotation;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Elevator extends SubsystemBase {
  /** Creates a new Elevator. */

  private enum ControlMode {
    kOpenLoop, kPID
  }

  private ControlMode m_controlMode = ControlMode.kOpenLoop;
  private final PIDController m_pidController = new PIDController(1,0,0 );
  private double m_demand;
  private final boolean m_isInverted;
  private final TalonFX m_motor;
  double outputVoltage = 0;

  public Elevator(int motorid, boolean isInverted){
    
    m_motor = new TalonFX(motorid);
    m_motor.setInverted(isInverted);
    m_motor.getVelocity().setUpdateFrequency(50);
    m_motor.getMotorVoltage().setUpdateFrequency(50);
    m_motor.optimizeBusUtilization();  
    m_motor.setNeutralMode(NeutralModeValue.Brake);

	  TalonFXConfiguration config = new TalonFXConfiguration();


    m_isInverted = isInverted;

	  config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    config.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
    config.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
	  config.SoftwareLimitSwitch.ReverseSoftLimitThreshold = 5;
    config.SoftwareLimitSwitch.ForwardSoftLimitThreshold = 20;
    config.CurrentLimits.SupplyCurrentLimitEnable = true;
    config.CurrentLimits.SupplyCurrentLimit = 40;



    SmartDashboard.putNumber("SP kP", 1);
    SmartDashboard.putNumber("SP kI", 0);
    SmartDashboard.putNumber("SP kD", 0);
    SmartDashboard.putNumber("SP kF", 0.1);
  }

  public double getPosition() {
    return m_motor.getPosition().getValueAsDouble();
  }


  @Override
  public void periodic() {
    m_motor.setNeutralMode(NeutralModeValue.Brake);
    // This method will be called once per scheduler run

    switch (m_controlMode) {

      case kOpenLoop:
        // Do openloop stuff here
        if (m_motor.getPosition().getValueAsDouble() < 5 && m_demand < 0){
         outputVoltage = 0;
        } else if (m_motor.getPosition().getValueAsDouble() > 80 && m_demand > 0){
          outputVoltage = 0;
        } else {
          outputVoltage = m_demand;
        }
        
        break;
    
      case kPID:
      m_pidController.setP(0.2);
      m_pidController.setI(0);
      m_pidController.setD(0);
      double kF = 0.002;
  
      outputVoltage = kF * m_demand + m_pidController.calculate(getPosition(), m_demand);
  
    
        break;
        default:
        // What happened!?
        break;
      }

      	// Publish to smart dashboard
        // SmartDashboard.putNumber("elevator " + (m_isInverted ? "Left" : "Right")+" Velocity", getVelocity());
        SmartDashboard.putNumber("elevator "+ (m_isInverted ? "Left" : "Right") +  " output voltage", outputVoltage);
        // SmartDashboard.putNumber("elevator "+ (m_isInverted ? "Left" : "Right") + " RPM Error", getErrorRPM());
        // SmartDashboard.putNumber("elevator " + (m_isInverted ? "Left" : "Right") + " RPM Percent Error", getErrorPercent());
        SmartDashboard.putString("elevator "+ (m_isInverted ? "Left" : "Right")+ " Control Mode", m_controlMode.toString());
        // SmartDashboard.putBoolean("elevator InRange"+ (m_isInverted ? "Left" : "Right"), isErrorInRange());
        SmartDashboard.putNumber("elevator "+ (m_isInverted ? "Left" : "Right")+ "elevator rotation", m_motor.getPosition().getValueAsDouble());
        SmartDashboard.putNumber("demand", m_demand);
		    SmartDashboard.putNumber("kP", 0);
		    SmartDashboard.putNumber("kI", 0);
		    SmartDashboard.putNumber("kD", 0);
		    SmartDashboard.putNumber("kF", 0);


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
	  return openLoopCommand(()-> OutputVoltage);
  }

  public void stop() {
	  setOutputVoltage(0);
  }

  //pid
  public double getErrorRotation(){
    if (m_controlMode == ControlMode.kPID){
    return m_pidController.getPositionError();
    }
    return 0;
  }
  public boolean isErrorInRange() {
    return (-4 < this.getErrorPercent() && this.getErrorPercent() < 4);
  }
  
  public double getErrorPercent(){
    if (m_controlMode == ControlMode.kPID){
    return (m_demand - getPosition()) / m_demand * 10;
    }
    return 0;
  }
  public Command waitUntilErrorInrange(){
    return Commands.waitUntil(()-> this.isErrorInRange());
    }
    
    public boolean isErrorOutOfRange() {
      return (this.getErrorPercent() > 15);
    }
    
    public Command waitUntilErrorOutOfRange(){
    return Commands.waitUntil(() -> this.isErrorOutOfRange());
    
    }
    public void setPIDSetpoint(double rotation) {
      m_controlMode = ControlMode.kPID;
      m_demand = rotation;
    }

    public Command pidCommand(DoubleSupplier rpmSupplier){
      return Commands.runEnd(
      () -> this.setPIDSetpoint(rpmSupplier.getAsDouble()), this::stop, this);
    }
    
    public Command pidCommand(double rotation){
      return pidCommand(() -> rotation);
    }

}
