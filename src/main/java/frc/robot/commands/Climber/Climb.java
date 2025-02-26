// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Climber;

import static edu.wpi.first.units.Units.Degrees;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.sysClimber;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class Climb extends Command {
  /** Creates a new Climb. */
  private sysClimber ClimberSys;
  private Angle PosRequest;

  /**
   * Command to move the climber system
   * @param subsystem Climber subsystem reference
   * @param position  Commanded postion degrees from home - must be larger than the current position
   */
  public Climb(sysClimber subsystem, Angle position) {
    // Use addRequirements() here to declare subsystem dependencies.
    ClimberSys = subsystem;
    PosRequest = position;

    addRequirements(ClimberSys);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    ClimberSys.setClimbPos(PosRequest);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    ClimberSys.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
