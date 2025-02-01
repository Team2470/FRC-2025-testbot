package frc.robot.subsystems;


import com.ctre.phoenix6.hardware.CANdi;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.controls.*;
import com.ctre.phoenix6.configs.*;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.jni.PlatformJNI;
import com.ctre.phoenix6.sim.DeviceType;
import com.ctre.phoenix6.sim.CANdiSimState;
import com.ctre.phoenix6.*;
import com.ctre.phoenix6.spns.*;
import com.ctre.phoenix6.signals.*;
import edu.wpi.first.units.*;
import edu.wpi.first.units.measure.*;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj.motorcontrol.Talon;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.hardware.TalonFXS;

import static edu.wpi.first.units.Units.*;

import java.lang.reflect.Array;
import java.util.function.DoubleSupplier;
public class CoralThing_FXS  extends SubsystemBase {
    private final CANdi CoralIntake;
    private final VoltageOut m_voltageOut = new VoltageOut(0);
    private TalonFXSConfiguration configs_FXS;
    
    private TalonFXS TalonFXS0;
    public CoralThing_FXS (int canDIid, int deviceId, boolean isInverted) {
        
        TalonFXS0 = new TalonFXS(deviceId, "Canivore");
        configs_FXS = new TalonFXSConfiguration();
        configs_FXS.Slot0.kP = 1;
        configs_FXS.Slot0.kI = 0;
        configs_FXS.Slot0.kD = 10;
        configs_FXS.Slot0.kV = 2;
        configs_FXS.CurrentLimits.SupplyCurrentLimitEnable = true;
        configs_FXS.CurrentLimits.SupplyCurrentLimit = 20;
        configs_FXS.Commutation.MotorArrangement = MotorArrangementValue.NEO550_JST;
        configs_FXS.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        
        
        CoralIntake = new CANdi(canDIid, "Canivore");///////////////

        CANdiConfiguration configs_CANdi = new CANdiConfiguration();
        
        configs_CANdi.DigitalInputs.S1CloseState = S1CloseStateValue.CloseWhenLow;
        configs_CANdi.DigitalInputs.S1FloatState = S1FloatStateValue.PullHigh;
        configs_CANdi.DigitalInputs.S2CloseState = S2CloseStateValue.CloseWhenLow;
        configs_CANdi.DigitalInputs.S2FloatState = S2FloatStateValue.PullLow;
        CoralIntake.getConfigurator().apply(configs_CANdi);
        CoralIntake.getS1Closed().setUpdateFrequency(100);
        CoralIntake.getS2Closed().setUpdateFrequency(100);
        
        //configs_FXS.HardwareLimitSwitch.ForwardLimitEnable = true;
        //configs_FXS.HardwareLimitSwitch.ForwardLimitSource = ForwardLimitSourceValue.RemoteCANdiS1;
        //configs_FXS.HardwareLimitSwitch.ForwardLimitRemoteSensorID = 0;
        

        TalonFXS0.getConfigurator().apply(configs_FXS);
        
    }
    public boolean getS1Closed() {
        return CoralIntake.getS1Closed().getValue(); 
    }
    public boolean getS2Closed() {
        return CoralIntake.getS2Closed().getValue();
    }
    
    public void stop() {
        TalonFXS0.stopMotor();
    }
    public boolean isButtonPressed(boolean button) {
        if (button) {
            return true;
        } else {
            return false;
        }
        
    }
    
    public double getPos() {
        return TalonFXS0.getPosition().getValueAsDouble();
    }
    public double getVelo() {
        return TalonFXS0.getVelocity().getValueAsDouble();
    }
    @Override
    public void periodic() {
        SmartDashboard.putBoolean("S1_Closed", getS1Closed());
        SmartDashboard.putBoolean("S2_Closed", getS2Closed());
        SmartDashboard.putNumber("position", getPos());
        SmartDashboard.putNumber("velocity", getVelo());
    }

    public Command runMotorForwardsCommand() {
	    return Commands.runEnd(
            
	        () -> TalonFXS0.setVoltage(1), this::stop, this);
        

    }
    public Command runMotorForwardsSpeedCommand(int motorVoltage) {
	    return Commands.runEnd(
            
	        () -> TalonFXS0.setVoltage(motorVoltage), this::stop, this);
        

    }
    public Command runMotorBackwardsSpeedCommand(int motorVoltage) {
	    return Commands.runEnd(
            
	        () -> TalonFXS0.setVoltage(-motorVoltage), this::stop, this);
        

    }

    public Command ignoreLimitCommand() {
	    return Commands.runEnd(
	       () ->{
            m_voltageOut.Output = 1;
            m_voltageOut.IgnoreHardwareLimits = true;
            TalonFXS0.setControl(m_voltageOut);

           }, this::stop, this);

    }

    //public Command setPositCommand() {
    //   return Commands.runEnd(()-> TalonFXS0.() - 30), this::stop, this);
    //}
    public Command slowMotorAtSensorCommand() {
        return new SequentialCommandGroup(
            runMotorForwardsSpeedCommand(3).until(()-> getS1Closed()),
            runMotorForwardsSpeedCommand(1)
            //runMotorBackwardsSpeedCommand(3)

            //setPositCommand()


        );
    }
    public Command stopMotorCommand() {
        return Commands.runEnd(() -> TalonFXS0.setVoltage(0), this::stop, this);
    }
}