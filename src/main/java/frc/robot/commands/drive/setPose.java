// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drive;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.sysDrive;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class setPose extends InstantCommand {
  private sysDrive Drive;
  private Distance pos_x, pos_y;
  private Angle rot;

  public setPose(sysDrive drive_sys, Distance x_pos, Distance y_pos, Angle deg) {
    // Use addRequirements() here to declare subsystem dependencies.
    Drive = drive_sys;
    pos_x = x_pos;
    pos_y = y_pos;
    rot = deg;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    Drive.setPose(pos_x, pos_y, rot);
  }
}
