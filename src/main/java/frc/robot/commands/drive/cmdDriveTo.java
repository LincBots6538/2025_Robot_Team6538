// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drive;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.kDrive;
import frc.robot.subsystems.sysDrive;


/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class cmdDriveTo extends Command {
  private sysDrive Drive;
  private double MAX_SPEED = kDrive.MAX_AUTO_SPEED.in(MetersPerSecond);
  private double kp = 1.0;
  private double xVec, yVec, dis, goal, pterm, drot, rot_cmd;
  private Pose2d current, target, delta;

  /** Creates a new cmdDriveTo.
   * Requires a stop cmd to Stop
   */
  public cmdDriveTo(sysDrive subsystem, Distance xWayPoint, Distance yWayPoint, Rotation2d Facing, Distance StopWithin) {
    // Use addRequirements() here to declare subsystem dependencies.
    Drive = subsystem;
    target = new Pose2d(xWayPoint, yWayPoint, Facing);
    goal = StopWithin.in(Meters);
    

    addRequirements(subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    current = Drive.getPose();
    delta = target.relativeTo(current);

    dis = delta.getTranslation().getDistance(Translation2d.kZero);

    pterm = MathUtil.clamp(kp*dis, -1, 1);
    xVec = MAX_SPEED * pterm * delta.getX()/dis;
    yVec = MAX_SPEED * pterm * delta.getY()/dis;

    // // rotation
    // drot = delta.getRotation().getRadians();
    // rot_cmd = 3 * MathUtil.clamp(drot/3, -1, 1);;
    // Drive.FCdrive(yVec, xVec, rot_cmd);

    // Rotation
    Drive.FCdrive_facing(xVec, yVec, target.getRotation());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (dis < goal) return true;
  
    return false;
  }
}
