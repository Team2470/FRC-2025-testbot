// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/** Add your docs here. */
public class Constants {

    public static class ElevatorConstants {

        public static final int kMotorID = 1;
        public static final int kMotorFollowerID = 2;
        public static final boolean kMotorInverted = true;
        public static final double kP = 2;
        public static final double kI = 0;
        public static final double kD = 0;
        public static final double kS = 0;
        public static final double kG = 0.235;
        public static final double kV = 0.22;
        public static final double kA = 0.0015;
        public static final double kUpLimit = 0;
        public static final double kDownLimit = 0;
        public static final int kRetractLimitChannel = 0;

        public static final double kRotationToInches = 2.0 * 1.0/20.0 * 1.751 * Math.PI;//TODO: Find the correct value
    }

    public static class WristConstants {

        public static final int kMotorID =  30;
        public static final boolean kMotorInverted = false;
        public static final int kEncoderID = 30;
        public static final double kSensorToMechanismRatio = 1.0;
        public static final double kRotorToSensorRatio = 160;
        public static final double kP = 0;
        public static final double kI = 0;
        public static final double kD = 0;
        public static final double kS = 0;
        public static final double kG = 0;
        public static final double kV = 0;
        public static final double kA = 0;
    
    }
    
    public static class ArmConstants {

        public static final int kMotorID =  30;
        public static final boolean kMotorInverted = false;
        public static final int kCANdiID = 30;
        public static final double kSensorToMechanismRatio = 1.0;
        public static final double kRotorToSensorRatio = 160;
        public static final double kP = 0;
        public static final double kI = 0;
        public static final double kD = 0;
        public static final double kS = 0;
        public static final double kG = 0;
        public static final double kV = 0;
        public static final double kA = 0;
    
    } 


}
