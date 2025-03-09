// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Auto;


import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.drive.TeleOpDrive;
import frc.robot.subsystems.sysDrive;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class LineAuto extends SequentialCommandGroup {
  /** Creates a new LineAuto. */
  

  public LineAuto(sysDrive drive) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
        new SequentialCommandGroup(
        new ParallelDeadlineGroup(
        new WaitCommand(2),
        new TeleOpDrive(drive, drive::autospeed, drive::zerospeed, drive::zerospeed)),
        new TeleOpDrive(drive, drive::zerospeed, drive::zerospeed, drive::zerospeed)
         )
    );
  }
}
