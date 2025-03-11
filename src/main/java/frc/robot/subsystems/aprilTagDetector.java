// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
public class aprilTagDetector extends SubsystemBase {
  /** Creates a new aprilTagDetector. */
  public aprilTagDetector() {}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
  

public class LimelightAprilTag {
    NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
    NetworkTableEntry tv = table.getEntry("tv");
    NetworkTableEntry tx = table.getEntry("tx");
    NetworkTableEntry ty = table.getEntry("ty");
    NetworkTableEntry ta = table.getEntry("ta");
    NetworkTableEntry tid = table.getEntry("tid");

    public void detectAprilTag() {
        // Check if the Limelight sees a target
        double targetValid = tv.getDouble(0.0);

        if (targetValid == 1.0) {
            // Get the horizontal and vertical offset from the target
            double x = tx.getDouble(0.0);
            double y = ty.getDouble(0.0);

            // Get the area of the target
            double area = ta.getDouble(0.0);
          
            // Get the id of the target
            double id = tid.getDouble(0.0);

            // Print the values
            System.out.println("Target X: " + x);
            System.out.println("Target Y: " + y);
            System.out.println("Target Area: " + area);
            System.out.println("Target ID: " + id);
        } else {
            System.out.println("No target detected");
        }
    }
}

}
