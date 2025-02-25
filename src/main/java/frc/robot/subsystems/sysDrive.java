// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;


import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.Telemetry;
import frc.robot.generated.TunerConstants;


public class sysDrive extends SubsystemBase {

  private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
  private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity

  private final SwerveRequest.FieldCentric FCdrive = new SwerveRequest.FieldCentric()
          .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
          .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
  private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
  private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

  public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();

  /** Creates a new sysDrive. */
  public sysDrive() {
    // Construct intial Powse
    setPose(Feet.of(0), null, null);
  }

  @Override
  public void periodic() {

  }

  /**
   * Reset the current Pose to the provide location & oreintation
   * 
   * @param pos_X   X position where Blue allaince wall is x=0
   * @param pos_y   Y postion where scoring table wall is y=0
   * @param rot
   */
  public void setPose(Distance pos_X, Distance pos_y, Angle rot){
    
    drivetrain.resetPose(new Pose2d(
      pos_X, 
      pos_y, 
      new Rotation2d(rot)));
  }

  /**
   * Sets wheel modules into 'X' Pattern to resist being pushed
   */
  public void brake(){
    drivetrain.setControl(brake);
  }

  /**
   * Rotate the chassis to face the commanded direction
   * 
   * @param deg   Direction to face
   */
  public void point(Double deg){
    Rotation2d rotation = Rotation2d.fromDegrees(deg);
    drivetrain.setControl(point.withModuleDirection(rotation));
  }

  /**
   * Field Centric Control moves robot relative to starting orientation
   * 
   * @param vel_X - Velocity away from operator, meters per second
   * @param vel_Y - Velocity from right to left, meters per second
   * @param rot - Positive Values counterclockwise, radians per second
   */
  public void FCdrive(double vel_X, double vel_Y, double rot){
    drivetrain.setControl(FCdrive
      .withVelocityX(vel_X)
      .withVelocityY(vel_Y)
      .withRotationalRate(rot)
    );
  }

  /*
   * Reset Rotation reference
   */
  public void resetFC(){
    drivetrain.seedFieldCentric();
  }

  /**
   * Returns the current Pose of the Robot
   * 
   * @return    The current Pose / position of the robot
   */
  public Pose2d getPose(){
    return drivetrain.getState().Pose;
  }

  public void registerTelemetry(Telemetry logger){
    drivetrain.registerTelemetry(logger::telemeterize);
  }

  public void SysID_dynamic(Direction dir){
    drivetrain.sysIdDynamic(dir);
  }

  public Command CmdSysID_dynamic(Direction dir){
    return this.runOnce(() -> SysID_dynamic(dir));
  }

  public void SysID_static(Direction dir){
    drivetrain.sysIdQuasistatic(dir);
  }

  public Command CmdSysID_static(Direction dir){
    return this.runOnce(() -> SysID_static(dir));
  }
}
