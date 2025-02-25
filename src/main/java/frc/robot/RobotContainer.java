// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

// erase this scott
package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.sysArm;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.Constants.kControllers;
import frc.robot.commands.Arm.ArmDrivePos;
import frc.robot.commands.drive.TeleOpDrive;
import frc.robot.commands.drive.cmdDriveTo;
import frc.robot.generated.TunerConstants;

import frc.robot.subsystems.sysDrive;

public class RobotContainer {
    
    private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private final Telemetry logger = new Telemetry(MaxSpeed);
    
    // Declare Subsystems
    private sysDrive sys_drive = new sysDrive();
    // Arm System
    private sysArm sys_Arm = new sysArm();
    // Elevator System
    // Climber System
    
    // Declare Controllers
    private final CommandXboxController jyst_Drive = new CommandXboxController(kControllers.DRIVE_PORT);
    private final CommandXboxController jyst_Manip = new CommandXboxController(kControllers.MANIP_PORT);

    // Auto Chooser
    private SendableChooser<Command> dsh_selAuto = new SendableChooser<>();
    
    //#region CTRE Swerve Control

    // //private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    // private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity

    // /* Setting up bindings for necessary control of the swerve drive platform */
    // private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
    //         .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
    //         .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
    // private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    // private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

    // //private final Telemetry logger = new Telemetry(MaxSpeed);

    // private final CommandXboxController joystick = new CommandXboxController(0);

    // public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();
    //#endregion

    public RobotContainer() {
        // Start Camera

        // Default Commands
        sys_drive.setDefaultCommand(new TeleOpDrive(
            sys_drive,
            jyst_Drive::getLeftY,
            jyst_Drive::getLeftX,
            jyst_Drive::getRightX));

        // 
        configureBindings();
    }

    private void configureBindings() {
        
        //#region CTRE Swerve Control

        // // Note that X is defined as forward according to WPILib convention,
        // // and Y is defined as to the left according to WPILib convention.
        // drivetrain.setDefaultCommand(
        //     // Drivetrain will execute this command periodically
        //     drivetrain.applyRequest(() ->
        //         drive.withVelocityX(-joystick.getLeftY() * MaxSpeed) // Drive forward with negative Y (forward)
        //             .withVelocityY(-joystick.getLeftX() * MaxSpeed) // Drive left with negative X (left)
        //             .withRotationalRate(-joystick.getRightX() * MaxAngularRate) // Drive counterclockwise with negative X (left)
        //     )
        // );

        // joystick.a().whileTrue(drivetrain.applyRequest(() -> brake));
        // joystick.b().whileTrue(drivetrain.applyRequest(() ->
        //     point.withModuleDirection(new Rotation2d(-joystick.getLeftY(), -joystick.getLeftX()))
        // ));

        // // Run SysId routines when holding back/start and X/Y.
        // // Note that each routine should be run exactly once in a single log.
        // joystick.back().and(joystick.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
        // joystick.back().and(joystick.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
        // joystick.start().and(joystick.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
        // joystick.start().and(joystick.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

        // // reset the field-centric heading on left bumper press
        // joystick.leftBumper().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));

        // drivetrain.registerTelemetry(logger::telemeterize);

        //#endregion

        // Run SysId routines when holding back/start and X/Y.
        // Note that each routine should be run exactly once in a single log.
        jyst_Drive.back().and(jyst_Drive.y()).whileTrue(sys_drive.CmdSysID_dynamic(Direction.kForward));
        jyst_Drive.back().and(jyst_Drive.x()).whileTrue(sys_drive.CmdSysID_dynamic(Direction.kReverse));
        jyst_Drive.start().and(jyst_Drive.y()).whileTrue(sys_drive.CmdSysID_static(Direction.kForward));
        jyst_Drive.start().and(jyst_Drive.x()).whileTrue(sys_drive.CmdSysID_static(Direction.kReverse));

        sys_drive.registerTelemetry(logger);

        jyst_Manip.y().onTrue(new ArmDrivePos(sys_Arm,0));
        jyst_Manip.x().onTrue(new ArmDrivePos(sys_Arm,160));
    }

    public Command getAutonomousCommand() {
        return Commands.print("No autonomous command configured");
    }
}
