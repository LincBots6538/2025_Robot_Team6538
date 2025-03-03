// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Arm;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.GlobalVariables;
import frc.robot.Constants.kArm;
import frc.robot.Constants.kElevator;
import frc.robot.subsystems.sysArm;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ArmDrivePos extends InstantCommand {
  private sysArm Arm;
  private double cmdPos;

  public ArmDrivePos(sysArm subsystem, double deg) {
    // Use addRequirements() here to declare subsystem dependencies.
    Arm = subsystem;
    cmdPos = deg;

    addRequirements(subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    double min = kArm.ARM_MIN;
    double max = kArm.ARM_MAX;
    if (GlobalVariables.Elevator_Position > kElevator.UPPER_LIMIT) min = kArm.ARM_HIGH_LIMIT;
    if (GlobalVariables.Elevator_Position < kElevator.LOWER_LIMIT) max = kArm.ARM_LOW_LIMIT;
    cmdPos = MathUtil.clamp(cmdPos, min, max);

    Arm.setArmPos(cmdPos);
  }
}
