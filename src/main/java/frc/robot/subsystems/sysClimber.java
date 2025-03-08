// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.kClimber;

public class sysClimber extends SubsystemBase {
  /** Creates a new sysClimber. */

  // Declare Motor Controllers
  private TalonFX mtrLeftClimb = new TalonFX(kClimber.LEFT_CANID);
  private TalonFX mtrRightClimb = new TalonFX(kClimber.RIGHT_CANID);

  // Configuration Objects
  private TalonFXConfiguration mtrCfg = new TalonFXConfiguration();
  private TalonFXConfiguration mtrCfg_right = new TalonFXConfiguration();
  
  // Closed Loop
  private PositionVoltage ClimbRequest = new PositionVoltage(0);



  public sysClimber() {
    // Set Motor Controller Parameters
    mtrCfg.Slot0
      .withKS(0)
      .withKP(kClimber.KP)
      .withKI(0)
      .withKD(0);
    mtrCfg.Voltage
        .withPeakForwardVoltage(Volts.of(8))
        .withPeakReverseVoltage(Volts.of(0));   // Ratch mech used in gearbox, backdriving is not possible
    mtrCfg.CurrentLimits
      .withStatorCurrentLimit(Amps.of(kClimber.CURRENT_LIMIT))
      .withStatorCurrentLimitEnable(true);
    mtrCfg.MotorOutput
      .withInverted(InvertedValue.Clockwise_Positive);
    mtrCfg.Feedback
      .withFeedbackSensorSource(FeedbackSensorSourceValue.RotorSensor)
      .withSensorToMechanismRatio(kClimber.RATIO);

    // Set Motor Controller Parameters
    mtrCfg_right.Slot0
      .withKS(0)
      .withKP(kClimber.KP)
      .withKI(0)
      .withKD(0);
    mtrCfg_right.Voltage
        .withPeakForwardVoltage(Volts.of(8))
        .withPeakReverseVoltage(Volts.of(0));   // Ratch mech used in gearbox, backdriving is not possible
    mtrCfg_right.CurrentLimits
      .withStatorCurrentLimit(Amps.of(kClimber.CURRENT_LIMIT))
      .withStatorCurrentLimitEnable(true);
    mtrCfg_right.MotorOutput
      .withInverted(InvertedValue.CounterClockwise_Positive);
    mtrCfg_right.Feedback
      .withFeedbackSensorSource(FeedbackSensorSourceValue.RotorSensor)
      .withSensorToMechanismRatio(kClimber.RATIO);

    // Apply Configuration, Set Postion to zero
    mtrLeftClimb.getConfigurator().apply(mtrCfg);
    mtrLeftClimb.setPosition(0);

    ClimbRequest.withSlot(0)
      .withLimitReverseMotion(true);

    // Set Right Motor to follow Left Motor, but inverted
    mtrRightClimb.getConfigurator().apply(mtrCfg_right);
    mtrRightClimb.setPosition(0);

    
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Climber Position", getClimbPos().in(Degrees));
    // limit motion beyond 190 degerees
  }

  /**
   * Set the Position of the Climber Arms
   * @param cmdPos  Commanded rotation from starting position
   */
  public void setClimbPos(Angle cmdPos){
    mtrLeftClimb.setControl(ClimbRequest.withPosition(cmdPos));
    mtrRightClimb.setControl(ClimbRequest.withPosition(cmdPos));
  }

  /**
   * Returns the current climber position
   * @return  Current climber position
   */
  public Angle getClimbPos(){
    return mtrLeftClimb.getPosition().getValue();
  }

  public void resetPos(){
    mtrLeftClimb.setPosition(0);
    mtrRightClimb.setPosition(0);
  }

  public void stop(){
    mtrLeftClimb.stopMotor();
    mtrRightClimb.stopMotor();
  }
}
