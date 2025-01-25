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
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static edu.wpi.first.units.Units.*;
public class CoralThing  extends SubsystemBase{
    private final CANdi CoralIntake;
    






    public CoralThing (int motorid) {
        CoralIntake = new CANdi(motorid, "Canivore");///////////////
        CANdiConfiguration configs = new CANdiConfiguration();
        CoralIntake.getConfigurator().apply(configs);
        
         

    }
    public boolean getS1Closed() {
        return CoralIntake.getS1Closed().getValue();
    }
}
