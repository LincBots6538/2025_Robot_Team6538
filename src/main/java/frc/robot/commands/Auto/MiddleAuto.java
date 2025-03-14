// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Auto;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.kArm;
import frc.robot.Constants.kElevator;
import frc.robot.commands.Arm.adjCoral;
import frc.robot.commands.Arm.setCoral;
import frc.robot.commands.Elevator.CombinedEleArm;
import frc.robot.commands.drive.Face;
import frc.robot.commands.drive.cmdDriveTo;
import frc.robot.commands.drive.setPose;
import frc.robot.subsystems.sysArm;
import frc.robot.subsystems.sysDrive;
import frc.robot.subsystems.sysElevator;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class MiddleAuto extends SequentialCommandGroup {
  /** Creates a new MiddleAuto. */
  public MiddleAuto(sysDrive Drive, sysArm Arm, sysElevator Elevator) {
    
    // Set Starting Position
    //Drive.setPose(Inches.of(60.75), Inches.of(158.5), Degrees.of(0));
    addCommands(
      new setPose(Drive,Inches.of(60.75), Inches.of(158.5), Degrees.of(0)),
      // Drive to left side of closest reef face
      new cmdDriveTo(Drive, 
        Inches.of(110), 
        Inches.of(152), 
        Rotation2d.fromDegrees(0), 
        true),
      new Face(Drive, Degrees.of(0)),
      new CombinedEleArm(Arm, Elevator, kArm.LVL4, kElevator.LVL_4),
      new setCoral(Arm),
      new cmdDriveTo(Drive, 
        Inches.of(117.688), 
        Inches.of(152), 
        Rotation2d.fromDegrees(0), 
        true),
      new adjCoral(Arm, 14),
      new WaitCommand(.5),
      new cmdDriveTo(Drive, 
        Inches.of(110), 
        Inches.of(152), 
        Rotation2d.fromDegrees(0), 
        true),
      new CombinedEleArm(Arm, Elevator, kArm.HOME, kElevator.HOME)
    );
  }
}
