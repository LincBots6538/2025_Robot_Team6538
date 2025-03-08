// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Arm;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.kArm;
import frc.robot.subsystems.sysArm;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class AutoIntake extends Command {
  private sysArm Arm;
  private Command PosCoral;
  /** Creates a new AutoIntake. */
  public AutoIntake(sysArm Arm_sys) {
    Arm = Arm_sys;
    
    // end command
    PosCoral = new SequentialCommandGroup(
      new WaitCommand(.1),
      new Rollers(Arm, 0));

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(Arm);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    Arm.setRollers(kArm.ROLLER_FWD);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    PosCoral.schedule();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (Arm.RollerLoaded()) return true;
    return false;
  }
}
