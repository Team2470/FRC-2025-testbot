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
        public static final double kP = 0.2/0.275046436822;
        public static final double kI = 0;
        public static final double kD = 0;
        public static final double kF = 0.002 / 0.275046436822;
        public static final double kUpLimit = 0;
        public static final double kDownLimit = 0;
        public static final int kRetractLimitChannel = 0;

        public static final double kRotationToInches = 1.0/20.0 * 1.751 * Math.PI;//TODO: Find the correct value
    }



}
