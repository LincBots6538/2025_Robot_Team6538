// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Elevator;

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
    // Check for potential collision
    // Will the Arm Target Collied if so pause Arm
    // Will the elevator Target Collide, if so pause elevator
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    curArm = Arm.getArmPos();
    curElevator = Elevator.getPosition();

    if ((curElevator > kElevator.UPPER_LIMIT) && (tgtArm < kArm.ARM_HIGH_LIMIT)) {
      Arm.setArmPos(curArm);
      Elevator.setPosition(tgtElevator);
    }

    if ((curArm < kArm.ARM_HIGH_LIMIT) && (tgtElevator > kElevator.UPPER_LIMIT)) {
      Arm.setArm(tgtArm);
      Elevator.setPosition(curElevator);
    }
    
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
