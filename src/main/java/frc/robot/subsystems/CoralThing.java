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
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.ctre.phoenix6.hardware.TalonFX;

import static edu.wpi.first.units.Units.*;

import java.lang.reflect.Array;
import java.util.function.DoubleSupplier;
public class CoralThing  extends SubsystemBase {
    private final CANdi CoralIntake;
    private final TalonFX m_motor;
    private final VoltageOut m_voltageOut = new VoltageOut(0);
    public CoralThing (int canDIid, int motorid, boolean isInverted) {
        CoralIntake = new CANdi(canDIid, "Canivore");///////////////
        m_motor = new TalonFX(motorid, "Canivore");
        m_motor.setInverted(isInverted);
        CANdiConfiguration configs = new CANdiConfiguration();
        CoralIntake.getConfigurator().apply(configs);
        TalonFXConfiguration config = new TalonFXConfiguration();
        CoralIntake.getS1Closed().setUpdateFrequency(100);
        CoralIntake.getS1Closed().setUpdateFrequency(100);
        config.HardwareLimitSwitch.ForwardLimitSource = ForwardLimitSourceValue.RemoteCANdiS1;
        config.HardwareLimitSwitch.ForwardLimitRemoteSensorID = 0;
        config.HardwareLimitSwitch.ForwardLimitEnable = true;
        m_motor.getConfigurator().apply(config); 
        m_motor.setNeutralMode(NeutralModeValue.Brake);
    }
    public boolean getS1Closed() {
        return CoralIntake.getS1Closed().getValue(); 
    }
    public boolean getS2Closed() {
        return CoralIntake.getS2Closed().getValue();
    }
    
    public void stop() {
        m_motor.stopMotor();
    }
    

    @Override
    public void periodic() {
        SmartDashboard.putBoolean("S1_Closed", getS1Closed());
        SmartDashboard.putBoolean("S2_Closed", getS2Closed());
    }

    public Command runMotorForwardsCommand() {
	    return Commands.runEnd(
	       () -> m_motor.setVoltage(6), this::stop, this);

    }

    public Command ignoreLimitCommand() {
	    return Commands.runEnd(
	       () ->{
            m_voltageOut.Output = 6;
            m_voltageOut.IgnoreHardwareLimits = true;
            m_motor.setControl(m_voltageOut);

           }, this::stop, this);

    }
}