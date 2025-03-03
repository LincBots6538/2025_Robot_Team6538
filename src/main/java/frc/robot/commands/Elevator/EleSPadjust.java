// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Elevator;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.sysElevator;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class EleSPadjust extends Command {
  private sysElevator elevator;
  private double sp_lead, cur, cmd;
  /** Creates a new EleSPadjust. */
  public EleSPadjust(sysElevator subsystem, double lead) {
    // Use addRequirements() here to declare subsystem dependencies.
    elevator = subsystem;
    sp_lead = lead;

    addRequirements(subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    cur = elevator.getPosition();
    cmd = cur + sp_lead;

    cmd = MathUtil.clamp(cmd, 0, 30.5);
    elevator.setPosition(cmd);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    elevator.setPosition(cur);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
