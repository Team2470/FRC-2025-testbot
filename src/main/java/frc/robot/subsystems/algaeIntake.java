package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.CANdi;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.hardware.TalonFXS;
import com.ctre.phoenix6.controls.*;
import com.ctre.phoenix6.controls.compound.*;
import com.ctre.phoenix6.configs.*;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.hardware.traits.*;
import com.ctre.phoenix6.jni.PlatformJNI;
import com.ctre.phoenix6.sim.DeviceType;
import com.ctre.phoenix6.sim.TalonFXSSimState;
import com.ctre.phoenix6.*;
import com.ctre.phoenix6.spns.*;
import com.ctre.phoenix6.signals.*;
import java.util.HashMap;
import edu.wpi.first.units.*;
import edu.wpi.first.units.measure.*;
import edu.wpi.first.wpilibj.motorcontrol.Talon;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class algaeIntake  extends SubsystemBase {
    private final CANdi algaeIntake;
    private TalonFXSConfiguration configs_FXS;
    private TalonFXS algaeTalonFXS1;

    public algaeIntake (int canDIid, int motorid, boolean isInverted) {
        algaeTalonFXS1 = new TalonFXS(motorid, "Canivore");
        
        configs_FXS = new TalonFXSConfiguration();
        configs_FXS.Slot0.kP = 1;
        configs_FXS.Slot0.kI = 0;
        configs_FXS.Slot0.kD = 10;
        configs_FXS.Slot0.kV = 2;
        configs_FXS.CurrentLimits.SupplyCurrentLimitEnable = true;
        configs_FXS.CurrentLimits.SupplyCurrentLimit = 20;
        configs_FXS.Commutation.MotorArrangement = MotorArrangementValue.NEO550_JST;
        configs_FXS.MotorOutput.NeutralMode = NeutralModeValue.Brake;

        algaeIntake = new CANdi(motorid, "Canivore");

        CANdiConfiguration configs_CANdi = new CANdiConfiguration();
        
        configs_CANdi.DigitalInputs.S1CloseState = S1CloseStateValue.CloseWhenLow;
        configs_CANdi.DigitalInputs.S1FloatState = S1FloatStateValue.PullHigh;
        configs_CANdi.DigitalInputs.S2CloseState = S2CloseStateValue.CloseWhenLow;
        configs_CANdi.DigitalInputs.S2FloatState = S2FloatStateValue.PullLow;
        algaeIntake.getConfigurator().apply(configs_CANdi);
        algaeIntake.getS1Closed().setUpdateFrequency(100);
        algaeIntake.getS2Closed().setUpdateFrequency(100);

        algaeTalonFXS1.getConfigurator().apply(configs_FXS);

        
    }
    public boolean getS1Closed() {
        return algaeIntake.getS1Closed().getValue(); 
    }
    public boolean getS2Closed() {
        return algaeIntake.getS2Closed().getValue();
    }
    
    public void stop() {
        algaeTalonFXS1.stopMotor();
    }
    public double getPos() {
        return algaeTalonFXS1.getPosition().getValueAsDouble();
    }
    public double getVelo() {
        return algaeTalonFXS1.getVelocity().getValueAsDouble();
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
            
	        () -> algaeTalonFXS1.setVoltage(1), this::stop, this);
        

    }
    public Command runMotorForwardsSpeedCommand(int motorVoltage) {
	    return Commands.runEnd(
            
	        () -> algaeTalonFXS1.setVoltage(motorVoltage), this::stop, this);
        

    }
    public Command runMotorBackwardsSpeedCommand(int motorVoltage) {
	    return Commands.runEnd(
            
	        () -> algaeTalonFXS1.setVoltage(-motorVoltage), this::stop, this);
        

    }
     public Command slowMotorAtSensorCommand() {
        return new SequentialCommandGroup(
            runMotorForwardsSpeedCommand(3).until(()-> getS1Closed()),
            runMotorForwardsSpeedCommand(1)
            //runMotorBackwardsSpeedCommand(3)

            //setPositCommand()


        );
    }
    public Command stopMotorCommand() {
        return Commands.runEnd(() -> algaeTalonFXS1.setVoltage(0), this::stop, this);
    }
}
