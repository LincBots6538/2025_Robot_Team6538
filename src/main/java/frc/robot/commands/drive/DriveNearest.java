// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drive;

import java.util.ArrayList;
import java.util.List;
import static edu.wpi.first.units.Units.*;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.sysDrive;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class DriveNearest extends InstantCommand {
  private sysDrive Drive;
  private List<Pose2d> reef = new ArrayList<>();
  private Pose2d near;

  public DriveNearest(sysDrive Drive_sys) {
    Drive = Drive_sys;
    // Use addRequirements() here to declare subsystem dependencies.
    
    
    reef.add(new Pose2d(Inches.of(132.58), Inches.of(108.960), Rotation2d.fromDegrees(60)));
    reef.add(new Pose2d(Inches.of(143.838), Inches.of(102.460), Rotation2d.fromDegrees(60)));
    reef.add(new Pose2d(Inches.of(193.537), Inches.of(102.460), Rotation2d.fromDegrees(120)));
    reef.add(new Pose2d(Inches.of(204.795), Inches.of(108.960), Rotation2d.fromDegrees(120)));
    reef.add(new Pose2d(Inches.of(229.644), Inches.of(152.000), Rotation2d.fromDegrees(180)));
    reef.add(new Pose2d(Inches.of(229.644), Inches.of(165.000), Rotation2d.fromDegrees(180)));
    reef.add(new Pose2d(Inches.of(204.795), Inches.of(208.040), Rotation2d.fromDegrees(-120)));
    reef.add(new Pose2d(Inches.of(193.537), Inches.of(214.540), Rotation2d.fromDegrees(-120)));
    reef.add(new Pose2d(Inches.of(143.838), Inches.of(214.540), Rotation2d.fromDegrees(-60)));
    reef.add(new Pose2d(Inches.of(132.580), Inches.of(208.040), Rotation2d.fromDegrees(-60)));
    reef.add(new Pose2d(Inches.of(107.731), Inches.of(165.000), Rotation2d.fromDegrees(0)));
    reef.add(new Pose2d(Inches.of(107.731), Inches.of(152.000), Rotation2d.fromDegrees(0)));

  } 

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    near = Drive.getPose().nearest(reef);
    Distance x_wp = near.getMeasureX();
    Distance y_wp = near.getMeasureY();
    Rotation2d face = near.getRotation();
    this.andThen(new cmdDriveTo(Drive, x_wp, y_wp, face, true));
  }
}
