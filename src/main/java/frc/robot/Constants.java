// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.FeetPerSecond;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.LinearVelocity;


/** Add your docs here. */
public final class Constants {
    public static class kControllers {
        public static final int DRIVE_PORT = 0;
        public static final int MANIP_PORT = 1;
    
        
    }
    public static class kDrive {
        public static final LinearVelocity MAX_SPEED = FeetPerSecond.of(13.0);
        public static final AngularVelocity TURNING_RATE = DegreesPerSecond.of(180.0);
        
    }

    public static class kArm{
        public static final double ARM_RATIO = 100; // 100:1 Versaplanetary
        public static final double HOME = 0; // Home is in a upwards position and angle for level 2 and 3
        public static final double TOP_OF_THE_REEF = 45; // For level 4 coral
        public static final double BALL = 160; // To remove algae
    }

    public static class kElevator{
        public static final double HOME = 0; // Home is when elevator is at bottom
        public static final double LVL_2 = 30; // For level 2 coral
        public static final double LVL_3 = 60; // For level 3 coral
        public static final double LVL_4 = 90; // For level 4 coral
    }

}
