package frc.robot.subsystems;

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
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static edu.wpi.first.units.Units.*;

public class TalonFXSTest extends SubsystemBase {
    private TalonFXSConfiguration configs;
    
    private TalonFXS TalonFXS0;
    public TalonFXSTest(int deviceId) {
            TalonFXS0 = new TalonFXS(deviceId, "Canivore");
            configs = new TalonFXSConfiguration();
            configs.Slot0.kP = 1;
            configs.Slot0.kI = 0;
            configs.Slot0.kD = 10;
            configs.Slot0.kV = 2;
            configs.CurrentLimits.SupplyCurrentLimitEnable = true;
            configs.CurrentLimits.SupplyCurrentLimit = 20;
            configs.Commutation.MotorArrangement = MotorArrangementValue.NEO550_JST;
            //configs.MotorOutput.NeutralMode = NeutralModeValue.Brake;
            TalonFXS0.getConfigurator().apply(configs);


        }

    public void stop() {
            TalonFXS0.stopMotor();
    }
    public double getPos() {
        return TalonFXS0.getPosition().getValueAsDouble();
    }
    public double getVelo() {
        return TalonFXS0.getVelocity().getValueAsDouble();
    }
    @Override
    public void periodic() {
        SmartDashboard.putNumber("position", getPos());
        SmartDashboard.putNumber("velocity", getVelo());
    }
    public Command runMotorForwardsCommand() {
        return Commands.runEnd (
            () -> TalonFXS0.setVoltage(1), this::stop, this);
    }
}
