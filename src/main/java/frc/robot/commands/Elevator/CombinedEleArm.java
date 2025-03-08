// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Elevator;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.kArm;
import frc.robot.Constants.kElevator;
import frc.robot.subsystems.sysArm;
import frc.robot.subsystems.sysElevator;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class CombinedEleArm extends Command {
  private sysArm Arm;
  private sysElevator Elevator;
  private double tgtArm, tgtElevator;
  private double errArm, errElevator;
  private double curArm, curElevator;
  private double cmdArm, cmdElevator;

  /** Creates a new CombinedEleArm. */
  public CombinedEleArm(sysArm Arm_sys, sysElevator Elevator_sys, double Arm_tgt, double Elevator_tgt) {
    Arm = Arm_sys;
    Elevator = Elevator_sys;
    tgtArm = Arm_tgt;
    tgtElevator = Elevator_tgt;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(Arm, Elevator);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // Clamp tgt to max positions
    tgtArm = MathUtil.clamp(tgtArm, kArm.ARM_MIN, kArm.ARM_MAX);
    tgtElevator = MathUtil.clamp(tgtElevator, kElevator.BOTTOM, kElevator.TOP);
    
    // Check for impossible tgts
    if ((tgtArm < kArm.ARM_HIGH_LIMIT) && (tgtElevator > kElevator.UPPER_LIMIT)) end(true);
    if ((tgtArm > kArm.ARM_LOW_LIMIT) && (tgtElevator < kElevator.LOWER_LIMIT)) end(true);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    curArm = Arm.getArmPos();
    curElevator = Elevator.getPosition();
    errArm = Math.abs(tgtArm - curArm);
    errElevator = Math.abs(tgtElevator - curElevator);

    cmdArm = tgtArm;
    cmdElevator = tgtElevator;

    // Arm going under elevator frame, Elevator first
    if ((curElevator > kElevator.UPPER_LIMIT) && (tgtArm < kArm.ARM_HIGH_LIMIT)) {
      cmdArm = curArm;
    }

    // Arm starting under Elevator frame, Arm first
    if ((curArm < kArm.ARM_HIGH_LIMIT) && (tgtElevator > kElevator.UPPER_LIMIT)) {
      cmdElevator = curElevator;
    }

    // Arm going low while elevator low, Elevator first
    if ((curElevator < kElevator.LOWER_LIMIT) && (tgtArm > kArm.ARM_LOW_LIMIT)) {
      cmdArm = curArm;
    }

    //  Elevator going down while arm is low, Arm first
    if ((curArm > kArm.ARM_LOW_LIMIT) && (tgtElevator < kElevator.LOWER_LIMIT)) {
      cmdElevator = curElevator;
    }
    

    Arm.setArmPos(cmdArm);
    Elevator.setPosition(cmdElevator);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if ((errArm < 2) && (errElevator < 1)) return true;
    return false;
  }
}
