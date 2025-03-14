// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Arm;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.sysArm;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class setRollers extends InstantCommand {
  private sysArm Arm;
  private double pwr;
  public setRollers(sysArm sysArm, double Power) {
    // Use addRequirements() here to declare subsystem dependencies.
    Arm = sysArm;
    pwr = Power;
    addRequirements(sysArm);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    Arm.setRollers(pwr);
  }
}
