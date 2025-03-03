// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Arm;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.sysArm;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class ArmSPadjust extends Command {
  /** Creates a new ArmSPadjust. */
  private sysArm Arm;
  private double sp_lead, cur, cmd;

  public ArmSPadjust(sysArm subsystem, double lead) {
    // Use addRequirements() here to declare subsystem dependencies.
    Arm = subsystem;
    sp_lead = lead;
    addRequirements(subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    cur = Arm.getArmPos();
    cmd = cur + sp_lead;

    cmd = MathUtil.clamp(cmd, -10, 160);
    Arm.setArmPos(cmd);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    Arm.setArmPos(cur);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
