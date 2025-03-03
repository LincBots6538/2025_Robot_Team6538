// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Elevator;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.sysArm;
import frc.robot.subsystems.sysElevator;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class resetEle extends InstantCommand {
  private sysElevator ele_sys;
  private sysArm arm_sys;

  public resetEle(sysElevator elevator, sysArm arm) {
    // Use addRequirements() here to declare subsystem dependencies.
    ele_sys = elevator;
    arm_sys = arm;

    addRequirements(elevator, arm);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    ele_sys.reset();
    arm_sys.reset();
  }
}
