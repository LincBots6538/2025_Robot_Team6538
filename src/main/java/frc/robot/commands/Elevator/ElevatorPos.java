// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Elevator;


import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.GlobalVariables;
import frc.robot.Constants.kElevator;
import frc.robot.subsystems.sysElevator;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class ElevatorPos extends Command {
  private sysElevator Ele;
  private double cmd_pos;
  
  

  public ElevatorPos(sysElevator ele_subsys, double ele_pos) {
    
    Ele = ele_subsys;
    cmd_pos = ele_pos;

    addRequirements(Ele);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    double max = kElevator.TOP;
    double min = kElevator.BOTTOM;
    if (GlobalVariables.Arm_Position < 38) max = kElevator.UPPER_LIMIT;
    if (GlobalVariables.Arm_Position > 150) min = kElevator.LOWER_LIMIT;
    cmd_pos = MathUtil.clamp(cmd_pos, min, max);

    Ele.setPosition(cmd_pos);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
