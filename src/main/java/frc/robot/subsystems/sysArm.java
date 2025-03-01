// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.DegreesPerSecondPerSecond;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.GlobalVariables;
import frc.robot.Constants.kArm;

public class sysArm extends SubsystemBase {
  /** Creates a new sysArm. */
  // use correct IDs
  private SparkMax mtrArm = new SparkMax(kArm.ARM_CANID, MotorType.kBrushless);
  private SparkMax mtrLeftRoll = new SparkMax(kArm.LEFT_ROLLER_CADID, MotorType.kBrushless);
  private SparkMax mtrRightRoll = new SparkMax(kArm.RIGHT_ROLLER_CANDID, MotorType.kBrushless);

  private SparkMaxConfig cfgArm = new SparkMaxConfig();
  private SparkMaxConfig cfgLeftRoll = new SparkMaxConfig();
  private SparkMaxConfig cfgRightRoll = new SparkMaxConfig();

  private DigitalInput coral_sw = new DigitalInput(0);
  
  private SparkClosedLoopController ctrArm;
  private RelativeEncoder encARM;

  private double Pos;

  public sysArm() {
    
    cfgArm.inverted(false)
      .idleMode(IdleMode.kBrake)
      .smartCurrentLimit(kArm.ARM_CURRENT_LIMIT)
      // Convert encoder values from motor rotations to mechanism degrees
      .encoder.positionConversionFactor(360.0/kArm.ARM_RATIO).velocityConversionFactor(360.0/kArm.ARM_RATIO); 
    cfgArm.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder)
      .pid(kArm.KP, kArm.KI, kArm.KD);
    // cfgArm.closedLoop.maxMotion
    //   .maxVelocity(kArm.MAX_SPEED.in(DegreesPerSecond)) // Arm degrees / s
    //   .maxAcceleration(kArm.MAX_ACCEL.in(DegreesPerSecondPerSecond)) // Arm Degrees / s /s
    //   .allowedClosedLoopError(10);

    mtrArm.configure(cfgArm, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    cfgLeftRoll.inverted(true)
      .idleMode(IdleMode.kBrake)
      .smartCurrentLimit(kArm.ROLLER_CURRENT_LIMIT);
    cfgRightRoll.idleMode(IdleMode.kBrake)
      .smartCurrentLimit(kArm.ROLLER_CURRENT_LIMIT)
      .follow(mtrLeftRoll, true);

    mtrLeftRoll.configure(cfgLeftRoll, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    mtrRightRoll.configure(cfgRightRoll, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    ctrArm = mtrArm.getClosedLoopController();
    encARM = mtrArm.getEncoder();
    encARM.setPosition(0);
    
    Pos = getArmPos();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    Pos = getArmPos();
    SmartDashboard.putNumber("Arm position", Pos);
    GlobalVariables.Arm_Position = Pos;
    SmartDashboard.putNumber("Elevator from arm", GlobalVariables.Elevator_Position);
    SmartDashboard.putNumber("Roller Current", mtrLeftRoll.getOutputCurrent());
    SmartDashboard.putBoolean("Coral Switch", coral_sw.get());

  }

  /**
   * Set Rollers to output power
   * @param pwr motor duty cycle -1.0 to 1.0
   */
  public void setRollers(double pwr){
    mtrLeftRoll.set(pwr);
  }

  public boolean RollerLoaded(){
    if (mtrLeftRoll.getOutputCurrent() > 1 )  return true;
    return false;
  }

  /**
   * Set postion of the arm
   *  
   * @param deg rotation of the arm relative the home postion in degrees
   */
  public void setArmPos(double deg){
    ctrArm.setReference(deg, ControlType.kPosition);
  }

  /**
   *  Returns the current position of the arm
   * @return  Arm position in degrees from home
   */
  public double getArmPos(){
    return encARM.getPosition();
  }

  public void setArm(double pwr){
    mtrArm.set(pwr);
  }
}
