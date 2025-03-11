// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Auto;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.drive.Face;
import frc.robot.commands.drive.cmdDriveTo;
import frc.robot.subsystems.sysDrive;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class MiddleAuto extends SequentialCommandGroup {
  /** Creates a new MiddleAuto. */
  public MiddleAuto(sysDrive Drive) {
    
    // Set Starting Position
    Drive.setPose(Inches.of(60.75), Inches.of(158.5), Degrees.of(0));
    addCommands(
      // Drive to left side of closest reef face
      new cmdDriveTo(Drive, 
        Inches.of(144.942), 
        Inches.of(104.371), 
        Rotation2d.fromDegrees(0), 
        true),
      new Face(Drive, Degrees.of(0)),
      // Deploy elevator
      new cmdDriveTo(Drive, 
        Inches.of(144.942), 
        Inches.of(104.371), 
        Rotation2d.fromDegrees(0), 
        true),
      // Score Coral
      // Drive back
      new cmdDriveTo(Drive, 
        Inches.of(144.942), 
        Inches.of(104.371), 
        Rotation2d.fromDegrees(0), 
        true)
    );
  }
}
