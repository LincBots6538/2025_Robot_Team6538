// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

// erase this scott
package frc.robot;

import static edu.wpi.first.units.Units.*;
// import frc.robot.subsystems.CommandSwerveDrivetrain;
// import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
// import com.ctre.phoenix6.swerve.SwerveRequest;
// import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.geometry.Rotation2d;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.Constants.*;
// import frc.robot.Constants.kClimber;
// import frc.robot.Constants.kControllers;
// import frc.robot.Constants.kDrive;
// import frc.robot.Constants.kElevator;
import frc.robot.commands.Arm.*;
// import frc.robot.commands.Arm.ArmSPadjust;
// import frc.robot.commands.Arm.ArmSetDC;
// import frc.robot.commands.Arm.Rollers;
// import frc.robot.commands.Arm.setRollers;
// import frc.robot.commands.Arm.AutoIntake;

// import frc.robot.commands.Auto.LineAuto;
// import frc.robot.commands.Auto.MiddleAuto;
import frc.robot.commands.Climber.Climb;
import frc.robot.commands.Elevator.*;
// import frc.robot.commands.Elevator.CombinedEleArm;
// import frc.robot.commands.Elevator.EleSPadjust;
// import frc.robot.commands.Elevator.ElevatorPos;
// import frc.robot.commands.Elevator.ElevatorPwr;
// import frc.robot.commands.Elevator.resetEle;
import frc.robot.commands.drive.*;
// import frc.robot.commands.drive.RCdrive;
// import frc.robot.commands.drive.TeleOpDrive;
// import frc.robot.commands.drive.reseedFC;
// import frc.robot.commands.drive.cmdDriveTo;

import frc.robot.commands.Auto.*;
import frc.robot.generated.TunerConstants;

import frc.robot.subsystems.sysDrive;
import frc.robot.subsystems.sysElevator;
import frc.robot.subsystems.sysArm;
import frc.robot.subsystems.sysClimber;

public class RobotContainer {
    
    private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private final Telemetry logger = new Telemetry(MaxSpeed);
    
    //#region Declare systems
    // Drive System
    private sysDrive sys_drive = new sysDrive();
    // Arm System
    private sysArm sys_Arm = new sysArm();
    // Elevator System
    private sysElevator sys_ele = new sysElevator();
    // Climber System
    private sysClimber sys_climb = new sysClimber();

    
    // Declare Controllers
    private final CommandXboxController jyst_Drive = new CommandXboxController(kControllers.DRIVE_PORT);
    private final CommandXboxController jyst_Manip = new CommandXboxController(kControllers.MANIP_PORT);
    private final CommandXboxController jyst_Test = new CommandXboxController(kControllers.TEST_PORT);

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

        // Auto Chooser
        dsh_selAuto.setDefaultOption("Line Auto", new LineAuto(sys_drive));
        dsh_selAuto.addOption("Left Auto", new LeftAuto(sys_drive, sys_Arm, sys_ele));
        dsh_selAuto.addOption("Middle Auto", new MiddleAuto(sys_drive, sys_Arm, sys_ele));
        dsh_selAuto.addOption("Right Auto", null);
        dsh_selAuto.addOption("Test", new SequentialCommandGroup( 
            new cmdDriveTo(sys_drive, Inches.of(0), Inches.of(0), Rotation2d.fromDegrees(0), true),
            new Face(sys_drive, Degrees.of(45))
            ));
        dsh_selAuto.addOption("Do Nothing", null);

        //Shuffleboard.getTab("Auto").add(dsh_selAuto);
        SmartDashboard.putData(dsh_selAuto);

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

        // Drive Controls
        // Default Command on jyst_Drivel left and right sticks
        jyst_Drive.povRight().whileTrue(new RCdrive(sys_drive, 0, (-1 * kDrive.JOG_SPEED.in(MetersPerSecond)), 0));
        jyst_Drive.povLeft().whileTrue(new RCdrive(sys_drive, 0, kDrive.JOG_SPEED.in(MetersPerSecond), 0));
        jyst_Drive.povDown().whileTrue(new RCdrive(sys_drive, (-1*kDrive.JOG_SPEED.in(MetersPerSecond)), 0, 0));
        jyst_Drive.povUp().whileTrue(new RCdrive(sys_drive, (1 * kDrive.JOG_SPEED.in(MetersPerSecond)), 0, 0));
        
        jyst_Drive.start().onTrue(new reseedFC(sys_drive));
        

        sys_drive.registerTelemetry(logger);
        
        // Arm buttons
        jyst_Manip.leftBumper().whileTrue(new Rollers(sys_Arm, kArm.ROLLER_BACK));        
        jyst_Manip.rightBumper().whileTrue(new Rollers(sys_Arm, kArm.ROLLER_FWD));
        
        // Roller buttons
        // Position Coral
        jyst_Manip.leftTrigger().onTrue(new setCoral(sys_Arm));
        // Auto Intake
        jyst_Manip.rightTrigger().onTrue(new AutoIntake(sys_Arm));
            
        
        // Elevator buttons
        jyst_Manip.a().onTrue(new CombinedEleArm(sys_Arm, sys_ele, kArm.HOME, kElevator.HOME)); // Home 
        jyst_Manip.b().onTrue(new CombinedEleArm(sys_Arm, sys_ele, kArm.LVL2, kElevator.LVL_2)); // Lvl 2
        jyst_Manip.x().onTrue(new CombinedEleArm(sys_Arm, sys_ele, kArm.LVL3, kElevator.LVL_3)); // Lvl 3
        jyst_Manip.y().onTrue(new CombinedEleArm(sys_Arm, sys_ele, kArm.LVL4, kElevator.LVL_4)); // Lvl 4

        // Manual Elevator
        jyst_Manip.povLeft().whileTrue(new ArmSPadjust(sys_Arm, -10.0));
        jyst_Manip.povRight().whileTrue(new ArmSPadjust(sys_Arm, 10.0));
        jyst_Manip.povDown().whileTrue(new EleSPadjust(sys_ele, -6.0));
        jyst_Manip.povUp().whileTrue(new EleSPadjust(sys_ele, 6.0));

        // Reset Elevator
        jyst_Manip.back().onTrue(new resetEle(sys_ele, sys_Arm));

        // Climb Buttons
        jyst_Manip.start().whileTrue(new Climb(sys_climb, kClimber.CLIMB_POS));     // Hold button to climb


        // Test Controller
        jyst_Test.x().whileTrue(new ArmSetDC(sys_Arm, -0.1));
        jyst_Test.y().whileTrue(new ArmSetDC(sys_Arm, 0.1));
        jyst_Test.a().whileTrue(new ElevatorPwr(sys_ele, -0.2));
        jyst_Test.b().whileTrue(new ElevatorPwr(sys_ele, 0.2));

        jyst_Test.leftBumper().whileTrue(new Rollers(sys_Arm, kArm.ROLLER_BACK));
        jyst_Test.rightBumper().whileTrue(new Rollers(sys_Arm, kArm.ROLLER_FWD));

        jyst_Test.povLeft().whileTrue(new RCdrive(sys_drive, 0, -1 * kDrive.JOG_SPEED.in(MetersPerSecond), 0));
        jyst_Test.povRight().whileTrue(new RCdrive(sys_drive, 0, kDrive.JOG_SPEED.in(MetersPerSecond), 0));
        jyst_Test.povUp().whileTrue(new RCdrive(sys_drive, kDrive.JOG_SPEED.in(MetersPerSecond), 0, 0));
        jyst_Test.povDown().whileTrue(new RCdrive(sys_drive, -1 * kDrive.JOG_SPEED.in(MetersPerSecond), 0, 0));
        
        // Run SysId routines when holding back/start and X/Y.
        // Note that each routine should be run exactly once in a single log.
        jyst_Test.back().and(jyst_Test.y()).whileTrue(sys_drive.CmdSysID_dynamic(Direction.kForward));
        jyst_Test.back().and(jyst_Test.x()).whileTrue(sys_drive.CmdSysID_dynamic(Direction.kReverse));
        jyst_Test.start().and(jyst_Test.y()).whileTrue(sys_drive.CmdSysID_static(Direction.kForward));
        jyst_Test.start().and(jyst_Test.x()).whileTrue(sys_drive.CmdSysID_static(Direction.kReverse));
    }

    public Command getAutonomousCommand() {

        return dsh_selAuto.getSelected();

    }
}
