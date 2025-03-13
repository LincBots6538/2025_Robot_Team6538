// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Auto;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.kArm;
import frc.robot.Constants.kElevator;
import frc.robot.commands.Arm.AutoIntake;
import frc.robot.commands.Arm.adjCoral;
import frc.robot.commands.Arm.setCoral;
import frc.robot.commands.Elevator.CombinedEleArm;
import frc.robot.commands.drive.Face;
import frc.robot.commands.drive.cmdDriveTo;
import frc.robot.subsystems.sysArm;
import frc.robot.subsystems.sysDrive;
import frc.robot.subsystems.sysElevator;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class RightAuto extends SequentialCommandGroup {
  /** Creates a new RightAuto. */
  public RightAuto(sysDrive Drive, sysArm Arm, sysElevator Elevator) {
    
    // Same as left, Y positions, and rotation flipped
    // Set Starting Position
    Drive.setPose(Inches.of(60.75), Inches.of(-18.25), Degrees.of(0));
    addCommands(
      // Set Starting Position (60.75, 18.25, 0R)
      // Drive to (145.817, 105.887, 60R)
      new cmdDriveTo(Drive, 
        Inches.of(144.942), 
        Inches.of(-104.371), 
        Rotation2d.fromDegrees(0), 
        true),
      new Face(Drive, Degrees.of(-60)),
      // Deploy Arm, Raise to LVL 4, commbinedEleArm / Adjust Coral
      new CombinedEleArm(Arm, Elevator, kArm.LVL4, kElevator.LVL_4),
      new setCoral(Arm),
      // Drive to (148.817, 111.083, 60R)
      new cmdDriveTo(Drive, 
        Inches.of(148.817), 
        Inches.of(-111.083), 
        Rotation2d.fromDegrees(-60), 
        true),
      // Score Coral
      new adjCoral(Arm, 14),
      // Drive to (145.817, 105.887, 60R)
      new cmdDriveTo(Drive, 
        Inches.of(144.942), 
        Inches.of(-104.371), 
        Rotation2d.fromDegrees(-60), 
        true),
      // Store Arm
      new CombinedEleArm(Arm, Elevator, kArm.HOME, kElevator.HOME),
      // Drive to near station
      new cmdDriveTo(Drive, 
        Inches.of(281.882), 
        Inches.of(-36.712), 
        Rotation2d.fromDegrees(-60), 
        true),
      new Face(Drive, Degrees.of(-126)),
      // Drive to Feeder Station & Intake
      new ParallelCommandGroup(
        // Start Intake
        new AutoIntake(Arm),
        // Drive to (286.437, 30.442, 126R)
        new cmdDriveTo(Drive, 
          Inches.of(286.437), 
          Inches.of(-30.442), 
          Rotation2d.fromDegrees(-126), 
          true)
      ),
      // Drive to (191.558, 105.887, 120R)
      new cmdDriveTo(Drive, 
        Inches.of(192.443), 
        Inches.of(-104.371), 
        Rotation2d.fromDegrees(-126), 
        true),
      new Face(Drive, Degrees.of(-120)),
      // Deploy Arm Raise to LVL 4
      new CombinedEleArm(Arm, Elevator, kArm.LVL4, kElevator.LVL_4),
      new setCoral(Arm),
      // Drive to (188.558, 111.083, 120R)
      new cmdDriveTo(Drive, 
        Inches.of(188.558), 
        Inches.of(-111.083), 
        Rotation2d.fromDegrees(-120), 
        true),
      // Score Coral
      new adjCoral(Arm, 14),
      // Drive to (191.558, 105.887, 120R)
      new cmdDriveTo(Drive, 
      Inches.of(192.443), 
      Inches.of(-104.371), 
        Rotation2d.fromDegrees(-120), 
        true),
      // Store Arm
      new CombinedEleArm(Arm, Elevator, kArm.HOME, kElevator.HOME),
      // Drive pre-feeder station
      new cmdDriveTo(Drive, 
        Inches.of(281.882), 
        Inches.of(-36.712), 
        Rotation2d.fromDegrees(-120), 
        true),
      new Face(Drive, Degrees.of(-126)),
      // Drive to Feeder Station & Intake
      new ParallelCommandGroup(
        // Start Intake
        new AutoIntake(Arm),
        // Drive to (286.437, 30.442, 126R)
        new cmdDriveTo(Drive, 
          Inches.of(286.437), 
          Inches.of(-30.442), 
          Rotation2d.fromDegrees(-126), 
          true)
      ),
      // Drive to (202.817, 112.387, 120R)
      new cmdDriveTo(Drive, 
        Inches.of(203.692), 
        Inches.of(-110.871), 
        Rotation2d.fromDegrees(-126), 
        true),
      new Face(Drive, Degrees.of(-120)),
      // Deploy Arm
      new CombinedEleArm(Arm, Elevator, kArm.LVL4, kElevator.LVL_4),
      new setCoral(Arm),
      // Drive to (199.817, 117.583, 120R)
      new cmdDriveTo(Drive, 
        Inches.of(199.817), 
        Inches.of(-117.583), 
        Rotation2d.fromDegrees(-120), 
        true),
      // Score Coral
      new adjCoral(Arm, 14),
      // Drive to (202.817, 112.387, 120R)
      new cmdDriveTo(Drive, 
        Inches.of(203.692), 
        Inches.of(-110.871), 
        Rotation2d.fromDegrees(-120), 
        true),
      // Store Arm
      new CombinedEleArm(Arm, Elevator, kArm.HOME, kElevator.HOME),
      // Drive pre-feeder station
      new cmdDriveTo(Drive, 
        Inches.of(281.882), 
        Inches.of(-36.712), 
        Rotation2d.fromDegrees(-120), 
        true),
      new Face(Drive, Degrees.of(-126)),
      // Drive to Feeder Station & Intake
      new ParallelCommandGroup(
        // Start Intake
        new AutoIntake(Arm),
        // Drive to (286.437, 30.442, 126R)
        new cmdDriveTo(Drive, 
          Inches.of(286.437), 
          Inches.of(-30.442), 
          Rotation2d.fromDegrees(-126), 
          true)
      )
    );
  }
}
