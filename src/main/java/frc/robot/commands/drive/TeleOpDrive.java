// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drive;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.kDrive;
import frc.robot.subsystems.sysDrive;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class TeleOpDrive extends Command {
  /** Creates a new TeleOpDrive. */
  private final sysDrive sys_drive;
  private final DoubleSupplier fwd_jyst, sid_jyst, rot_jyst;
  private double fwd_vel, left_vel, rot_spd, rot_fac;
  private Rotation2d rot_cmd;

  public TeleOpDrive(sysDrive drive, DoubleSupplier vel_fwd, DoubleSupplier vel_sid, DoubleSupplier rot) {
    // Use addRequirements() here to declare subsystem dependencies.
    sys_drive = drive;
    fwd_jyst = vel_fwd;
    sid_jyst = vel_sid;
    rot_jyst = rot;

    addRequirements(sys_drive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // Xbox controller  Y axis: Up = -1.0 / Down = +1.0
    //                  X Axis: Left = -1.0 / Right = +1.0
    
    fwd_vel = MathUtil.applyDeadband(fwd_jyst.getAsDouble(), 0.1);   // Apply deadband to joystick Value
    fwd_vel = Math.signum(fwd_vel) * Math.abs(Math.pow(fwd_vel, 2));        // Square the input
    fwd_vel = 1.0 * fwd_vel * kDrive.MAX_SPEED.in(MetersPerSecond);          // Invert Joystick value, Scale to Max Velocity

    left_vel = MathUtil.applyDeadband(sid_jyst.getAsDouble(), 0.1);  // Apply deadband to joystick value
    left_vel = Math.signum(left_vel) * Math.abs(Math.pow(left_vel, 2));     // Square the input
    left_vel = 1.0 * left_vel * kDrive.MAX_SPEED.in(MetersPerSecond);        // Invert Joystick value, Scale to Max Velocity

    rot_spd = MathUtil.applyDeadband(rot_jyst.getAsDouble(), 0.1);   // Apply deadband to joystick input
    rot_spd = Math.signum(rot_spd) * Math.abs(Math.pow(rot_spd, 2));        // Square the input
    rot_spd = -1.0 * rot_spd * kDrive.TURNING_RATE.in(RadiansPerSecond);      // Invert Joystick value, Scale to Max Velocity

    sys_drive.FCdrive(fwd_vel, left_vel, rot_spd);
    
    // Facing direction
    // rot_cmd = sys_drive.getPose().getRotation();
    // rot_fac = MathUtil.applyDeadband(rot_jyst.getAsDouble(), 0.1);   // Apply deadband to joystick input
    // rot_fac = Math.signum(rot_spd) * Math.abs(Math.pow(rot_spd, 2));        // Square the input
    // rot_fac = -1.0 * rot_fac * kDrive.STEERING_SP_LEAD.in(Degrees);      // Invert Joystick value, Scale to Max Velocity
    // rot_cmd = rot_cmd.plus(Rotation2d.fromDegrees(rot_fac));

    // sys_drive.FCdrive_facing(left_vel, fwd_vel, rot_cmd);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
