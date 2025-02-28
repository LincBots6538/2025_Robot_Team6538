// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;

import edu.wpi.first.units.measure.LinearAcceleration;
import edu.wpi.first.units.measure.LinearVelocity;


/** Add your docs here. */
public final class Constants {
    public static class kControllers {
        public static final int DRIVE_PORT = 0;
        public static final int MANIP_PORT = 1;
        public static final int TEST_PORT = 2;
    
        
    }
    public static class kDrive {
        public static final LinearVelocity MAX_SPEED = FeetPerSecond.of(13.0);
        public static final AngularVelocity TURNING_RATE = DegreesPerSecond.of(180.0);

        public static final LinearVelocity MAX_AUTO_SPEED = FeetPerSecond.of(5);
        public static final LinearVelocity JOG_SPEED = FeetPerSecond.of(2.0);
        
    }

    public static class kArm{
        public static final double ARM_RATIO = 100; // 100:1 Versaplanetary
        public static final double HOME = 0; // Home is in a upwards position and angle for level 2 and 3
        public static final double TOP_OF_THE_REEF = 45; // For level 4 coral
        public static final double BALL = 160; // To remove algae
        public static final double ROLLER_FWD = 0.3; 
        public static final double ROLLER_BACK = -0.3;

        public static final int ARM_CURRENT_LIMIT = 20;
        public static final int ROLLER_CURRENT_LIMIT = 10;

        public static final int LEFT_ROLLER_CADID = 4;
        public static final int RIGHT_ROLLER_CANDID = 5;
        public static final int ARM_CANID = 6;
        public static final double KP = 1.0;
        public static final double KI = 0.0;
        public static final double KD = 0.0;
        public static final AngularVelocity MAX_SPEED = DegreesPerSecond.of(180);
        public static final AngularAcceleration MAX_ACCEL = MAX_SPEED.div(Seconds.of(1));
        public static final Angle SP_ERROR = Degrees.of(1.0);
    }

    public static class kElevator{
        // these height should represent the distance the elevator moves from the starting position, or we can change the math to be the height from the ground
        public static final double HOME = 0; // Home is when elevator is at bottom
        public static final double LVL_2 = 31.89; // For level 2 coral (inches)
        public static final double LVL_3 = 47.64; // For level 3 coral (inches)
        public static final double LVL_4 = 72.05; // For level 4 coral (inches)
        
        public static final double RATIO = 49; // 49:1
        public static final double SPROCKET_DIA = 1.75; // Pitch Diameter of 22T #25 Sprocket
        public static final double MTR_TO_IN = 3.1415 * SPROCKET_DIA / RATIO;

        public static final int CURRENT_LIMIT = 30;
        public static final int LEFT_CANID = 2;
        public static final int RIGHT_CANID = 3;
        
        public static final double KP = 1.0;
        public static final double KI = 0.0;
        public static final double KD = 0.0;
        public static final LinearVelocity MAX_SPEED = InchesPerSecond.of(20);
        public static final LinearAcceleration MAX_ACCEL = MAX_SPEED.div(Seconds.of(1));
    }

    public static class kClimber {
    
        public static final double RATIO = 27.0 * 3.0;
        public static final double KP = .1;
        public static final double CURRENT_LIMIT = 40;
        public static final int LEFT_CANID = 8;
        public static final int RIGHT_CANID = 9;

        public static final Angle CLIMB_POS = Degrees.of(180);

    }
}
