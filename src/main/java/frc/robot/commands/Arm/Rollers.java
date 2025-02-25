// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Arm;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.sysArm;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class Rollers extends Command {
  private sysArm Roller;
  private double cmd_pwr;
  /** Creates a new Rollers. */
  public Rollers(sysArm subsystem, double mtr_pwr) {
    
  Roller = subsystem;
  cmd_pwr = mtr_pwr;

  addRequirements(Roller);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    Roller.setRollers(cmd_pwr); // turns rollers on immediatly when button is pressed
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    Roller.setRollers(0); // shuts rollers off
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
