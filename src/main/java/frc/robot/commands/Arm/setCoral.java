// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Arm;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.sysArm;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class setCoral extends Command {
  private sysArm Arm;
  private boolean coral_fg;
  /** Creates a new setCoral. */
  public setCoral(sysArm Arm_sys) {
    Arm = Arm_sys;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(Arm);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // if (Arm.coralSW()) end(true);
    Arm.setRollersDist(-6);
    coral_fg = Arm.coralSW();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (!coral_fg && Arm.coralSW()){
      Arm.setRollersDist(-2);
      return true;
    }
    return false;
  }
}
