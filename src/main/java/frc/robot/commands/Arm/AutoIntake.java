// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Arm;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.kArm;
import frc.robot.subsystems.sysArm;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class AutoIntake extends Command {
  private sysArm Arm;
  private Timer timer;

  /** Creates a new AutoIntake. Timing command, end when Coral has been detected */
  public AutoIntake(sysArm Arm_sys) {
    Arm = Arm_sys;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(Arm);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    timer.reset();
    timer.start();
    Arm.setRollers(kArm.ROLLER_FWD);

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (!Arm.coralSW()) {
      Arm.setRollersDist(3.5);
      end(false);
    } else if (timer.hasElapsed(10)) end(true);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    timer.stop();
    if (interrupted) Arm.setRollers(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    

    return false;
  }
}
