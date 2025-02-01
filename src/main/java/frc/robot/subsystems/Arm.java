package frc.robot.subsystems;


import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.CANdi;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.controls.*;
import com.ctre.phoenix6.configs.*;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.jni.PlatformJNI;
import com.ctre.phoenix6.sim.DeviceType;

import static edu.wpi.first.units.Units.Rotations;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.*;
import com.ctre.phoenix6.spns.*;
import com.ctre.phoenix6.signals.*;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.*;
import edu.wpi.first.units.measure.*;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.WristConstants;
import frc.robot.Constants.ArmConstants;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.ctre.phoenix6.hardware.TalonFX;

public class Arm extends SubsystemBase {
    private enum ControlMode {
        kOpenLoop, kPID, kStop, kHoming
    }
    //
    // Hardware
    //
    private final TalonFX m_motor;
    private final CANdi m_candi;

    //
    // State
    //
    private ControlMode m_controlMode = ControlMode.kStop;
    private double m_demand;

    private final ProfiledPIDController m_pidController = new ProfiledPIDController(ArmConstants.kP,
      ArmConstants.kI, ArmConstants.kD,
      new TrapezoidProfile.Constraints(0, 0));

    private ArmFeedforward m_feedforward = new ArmFeedforward(ArmConstants.kS, ArmConstants.kG,
      ArmConstants.kV,
      ArmConstants.kA);

    public Arm () {
        CANdiConfiguration candiconfig = new CANdiConfiguration();
        candiconfig.PWM1.AbsoluteSensorDiscontinuityPoint = 0.5;
        candiconfig.PWM1.SensorDirection = false;
        candiconfig.PWM1.AbsoluteSensorOffset = 0;
        m_candi = new CANdi(ArmConstants.kCANdiID, "rio");
        m_candi.getConfigurator().apply(candiconfig);
    
        TalonFXConfiguration motorConfig = new TalonFXConfiguration();
        motorConfig.Feedback.FeedbackRemoteSensorID = m_candi.getDeviceID();
        motorConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.FusedCANdiPWM1;
        motorConfig.Feedback.SensorToMechanismRatio = ArmConstants.kSensorToMechanismRatio;
        motorConfig.Feedback.RotorToSensorRatio = ArmConstants.kRotorToSensorRatio;
        
        motorConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        motorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;


        motorConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
        motorConfig.CurrentLimits.SupplyCurrentLimit = 40;
        motorConfig.CurrentLimits.StatorCurrentLimitEnable = false;
        motorConfig.CurrentLimits.StatorCurrentLimit = 125;

        motorConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
        motorConfig.SoftwareLimitSwitch.ForwardSoftLimitThreshold = .25;
        motorConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
        motorConfig.SoftwareLimitSwitch.ReverseSoftLimitThreshold = -.25;

        m_motor = new TalonFX(ArmConstants.kMotorID, "rio");
        m_motor.getConfigurator().apply(motorConfig);

        // We want to read position data from the leader motor
        m_motor.getPosition().setUpdateFrequency(50);
        m_motor.getVelocity().setUpdateFrequency(50);
        m_motor.optimizeBusUtilization();
    
        SmartDashboard.putNumber("Arm kP", ArmConstants.kP);
        SmartDashboard.putNumber("Arm kI", ArmConstants.kI);
        SmartDashboard.putNumber("Arm kD", ArmConstants.kD);
        SmartDashboard.putNumber("Arm kS", ArmConstants.kS);
        SmartDashboard.putNumber("Arm kG", ArmConstants.kG);
        SmartDashboard.putNumber("Arm kV", ArmConstants.kV);
        SmartDashboard.putNumber("Arm kA", ArmConstants.kA);

    }

    /**
     * 
     * @return Degrees
     */
    public double getPosition() {
        return Units.rotationsToDegrees(m_motor.getPosition().getValueAsDouble());
    }

    public double getVelocity() {
        return Units.rotationsToDegrees(m_motor.getVelocity().getValueAsDouble());
    }
    
    @Override
    public void periodic () {
        double outputVoltage = 0;
        switch (m_controlMode) {
            case kStop:
      
                outputVoltage = 0;
                break;
      
            case kOpenLoop:
                // Do openloop stuff here
        
                outputVoltage = m_demand;
      
              break;

            case kPID:
                
                m_feedforward = new ArmFeedforward(
                    SmartDashboard.getNumber("Arm kS", ArmConstants.kS),
                    SmartDashboard.getNumber("Arm kG", ArmConstants.kG),
                    SmartDashboard.getNumber("Arm kV", ArmConstants.kV),
                    SmartDashboard.getNumber("Arm kA", ArmConstants.kA)
                );

                m_pidController.setP(SmartDashboard.getNumber("Arm kP", ArmConstants.kP));
                m_pidController.setI(SmartDashboard.getNumber("Arm kI", ArmConstants.kI));
                m_pidController.setD(SmartDashboard.getNumber("Arm kD", ArmConstants.kD));
                
                double PIDoutPutVoltage = m_pidController.calculate(Units.degreesToRadians(getPosition()), Units.degreesToRadians(m_demand));

                double feedforwardVoltage = m_feedforward.calculate(m_pidController.getSetpoint().position,
                    m_pidController.getSetpoint().velocity);

                outputVoltage = PIDoutPutVoltage + feedforwardVoltage;
                SmartDashboard.putNumber("Arm Pid output voltage", PIDoutPutVoltage);
                SmartDashboard.putNumber("Arm Feed Fowrad output voltage", feedforwardVoltage);
                SmartDashboard.putNumber("Arm PID Profile Position",Units.radiansToDegrees(m_pidController.getSetpoint().position));
                SmartDashboard.putNumber("Arm PID Profile Velocity",Units.radiansToDegrees(m_pidController.getSetpoint().velocity));

                break;
        }

        SmartDashboard.putNumber("Arm Position", getPosition());
        SmartDashboard.putNumber("Arm Velocity", getVelocity());
        SmartDashboard.putString("Arm Controlmode", m_controlMode.toString());
        SmartDashboard.putNumber("Arm Demand", m_demand);

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

    //
    // PID
    //
    public double getErrorAngle(){
        if (m_controlMode == ControlMode.kPID){
            return Math.toDegrees(m_pidController.getPositionError());
        }
        return 0;
    }

    public boolean isAngleErrorInRange(){
        if (m_controlMode == ControlMode.kPID){
            return (-0.7 < getErrorAngle() && getErrorAngle() < 0.7);
        }
        return false;
    }
    
      public double getErrorPercent() {
        if (m_controlMode == ControlMode.kPID) {
          return (m_demand - getPosition()) / m_demand * 10;
        }
        return 0;
      }
    
      public Command waitUntilErrorInrange() {
        return Commands.waitUntil(() -> this.isAngleErrorInRange());
      }

      public void setPIDSetpoint(double degrees) {
        m_controlMode = ControlMode.kPID;
        m_demand = degrees;
      }

      public Command pidCommand(DoubleSupplier degrees) {
        return Commands.runEnd(
            () -> this.setPIDSetpoint(degrees.getAsDouble()), this::stop, this);
      }
    
      public Command pidCommand(double degrees) {
        return pidCommand(() -> degrees);
      }

}
