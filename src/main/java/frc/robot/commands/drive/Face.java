// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drive;

import static edu.wpi.first.units.Units.Degrees;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.sysDrive;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class Face extends Command {
  private sysDrive drive;
  private Rotation2d tgt_dir, cur_dir, err;

  /**  
   *  Creates a new Face. Drive command to face a given direction. Ends when err is less than 3 degrees
   * @param drive_sys Drive subsystem Object
   * @param faceDir Direction for the chassis to face
  */
  public Face(sysDrive drive_sys, Angle faceDir) {
    drive = drive_sys;
    tgt_dir = Rotation2d.fromDegrees(faceDir.in(Degrees));

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(drive_sys);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    cur_dir = drive.getPose().getRotation();
    //err = Math.abs(cur_dir - direction.in(Degrees));
    
    err = cur_dir.minus(tgt_dir);  // ABS? can this be negative

    drive.point(tgt_dir.getDegrees());

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    SmartDashboard.putString("last cmd", "face");
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (err.getDegrees() < 3) return true;
    return false;
  }
}
