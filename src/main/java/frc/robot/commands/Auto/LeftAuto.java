// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Auto;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.drive.Stop;
import frc.robot.commands.drive.cmdDriveTo;
import frc.robot.subsystems.sysArm;
import frc.robot.subsystems.sysDrive;
import frc.robot.subsystems.sysElevator;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class LeftAuto extends SequentialCommandGroup {
  /** Creates a new LeftAuto. */
  public LeftAuto(sysDrive Drive, sysArm Arm, sysElevator Elevator) {
   
    // Set Starting Position
    Drive.setPose(Inches.of(60.75), Inches.of(18.25), Degrees.of(0));
    addCommands(
      // Set Starting Position (60.75, 18.25, 0R)
      // Drive to (145.817, 105.887, 60R)
      new cmdDriveTo(Drive, 
        Inches.of(145.817), 
        Inches.of(105.887), 
        Rotation2d.fromDegrees(60), 
        Inches.of(6)),
      new Stop(Drive),
      // Deploy Arm, Raise to LVL 4
      // Drive to (148.817, 111.083, 60R)
      new cmdDriveTo(Drive, 
        Inches.of(148.817), 
        Inches.of(111.083), 
        Rotation2d.fromDegrees(60), 
        Inches.of(1)),
      new Stop(Drive),
      // Score Coral
      // Drive to (145.817, 105.887, 60R)
      new cmdDriveTo(Drive, 
        Inches.of(145.817), 
        Inches.of(105.887), 
        Rotation2d.fromDegrees(60), 
        Inches.of(1)),
      new Stop(Drive),
      // Store Arm
      // Drive to (286.437, 30.442, 126R)
      new cmdDriveTo(Drive, 
        Inches.of(286.437), 
        Inches.of(30.442), 
        Rotation2d.fromDegrees(126), 
        Inches.of(6)),
      new Stop(Drive),
      // Intake Coral w/detection
      // Drive to (191.558, 105.887, 120R)
      new cmdDriveTo(Drive, 
        Inches.of(191.558), 
        Inches.of(105.887), 
        Rotation2d.fromDegrees(120), 
        Inches.of(6)),
      new Stop(Drive),
      // Deploy Arm Raise to LVL 4
      // Drive to (188.558, 111.083, 120R)
      new cmdDriveTo(Drive, 
        Inches.of(188.558), 
        Inches.of(111.083), 
        Rotation2d.fromDegrees(120), 
        Inches.of(1)),
      new Stop(Drive),
      // Score Coral
      // Drive to (191.558, 105.887, 120R)
      new cmdDriveTo(Drive, 
        Inches.of(191.558), 
        Inches.of(105.887), 
        Rotation2d.fromDegrees(120), 
        Inches.of(1)),
      new Stop(Drive),
      // Store Arm
      // Drive to (286.437, 30.442, 126R)
      new cmdDriveTo(Drive, 
        Inches.of(286.437), 
        Inches.of(30.442), 
        Rotation2d.fromDegrees(126), 
        Inches.of(6)),
      new Stop(Drive),
      // Intake Coral w/detection
      // Drive to (202.817, 112.387, 120R)
      new cmdDriveTo(Drive, 
        Inches.of(202.817), 
        Inches.of(112.387), 
        Rotation2d.fromDegrees(120), 
        Inches.of(6)),
      new Stop(Drive),
      // Deploy Arm
      // Drive to (199.817, 117.583, 120R)
      new cmdDriveTo(Drive, 
        Inches.of(199.817), 
        Inches.of(117.583), 
        Rotation2d.fromDegrees(120), 
        Inches.of(1)),
      new Stop(Drive),
      // Score Coral
      // Drive to (202.817, 112.387, 120R)
      new cmdDriveTo(Drive, 
        Inches.of(202.817), 
        Inches.of(112.387), 
        Rotation2d.fromDegrees(120), 
        Inches.of(1)),
      new Stop(Drive),
      // Store Arm
      // Drive to (286.437, 30.442, 126R)
      new cmdDriveTo(Drive, 
        Inches.of(286.437), 
        Inches.of(30.442), 
        Rotation2d.fromDegrees(126), 
        Inches.of(6)),
      new Stop(Drive)
      // Intake Coral
    );
  }
}
