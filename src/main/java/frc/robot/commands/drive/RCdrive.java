// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drive;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.sysDrive;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class RCdrive extends Command {
  /** Creates a new RCdrive. */

  private sysDrive Drive;
  private double xVel, yVel, rot_cmd;
  public RCdrive(sysDrive subsystem, double vel_fwd, double vel_sid, double rot) {
    // Use addRequirements() here to declare subsystem dependencies.
    Drive = subsystem;
    xVel = vel_fwd;
    yVel = vel_sid;
    rot_cmd = rot;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // Drive.RCDrive(xVel, yVel, rot_cmd);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    Drive.RCDrive(xVel, yVel, rot_cmd);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    Drive.RCDrive(0,0,0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
