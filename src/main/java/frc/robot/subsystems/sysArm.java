// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

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

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.kArm;

public class sysArm extends SubsystemBase {
  /** Creates a new sysArm. */
  // use correct IDs
  private SparkMax mtrArm = new SparkMax(0, MotorType.kBrushless);
  private SparkMax mtrLeftRoll = new SparkMax(0, MotorType.kBrushless);
  private SparkMax mtrRightRoll = new SparkMax(0, MotorType.kBrushless);

  private SparkMaxConfig cfgArm = new SparkMaxConfig();
  private SparkMaxConfig cfgLeftRoll = new SparkMaxConfig();
  private SparkMaxConfig cfgRightRoll = new SparkMaxConfig();

  
  private SparkClosedLoopController ctrArm;
  private RelativeEncoder encARM;

  public sysArm() {
    
    cfgArm.inverted(true)
      .idleMode(IdleMode.kBrake)
      .smartCurrentLimit(kArm.ARM_CURRENT_LIMIT)
      .encoder.positionConversionFactor(1000).velocityConversionFactor(1000);
    cfgArm.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder)
      .pid(1.0, 0, 0);

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
    
      
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  /**
   * Set Rollers to output power
   * @param pwr motor duty cycle -1.0 to 1.0
   */
  public void setRollers(double pwr){
    mtrLeftRoll.set(pwr);
  }

  /**
   * Set postion of the arm
   *  
   * @param deg rotation of the arm relative the home postion in degrees
   */
  public void setArmPos(double deg){

    double mtrRot = deg/360 * kArm.ARM_RATIO;
    ctrArm.setReference(mtrRot, ControlType.kPosition);
  }

  /**
   *  Returns the current position of the arm
   * @return  Arm position in degrees from home
   */
  public double getArmPos(){
    double mtrROT = encARM.getPosition();
    return (mtrROT / kArm.ARM_RATIO) * (360);
  }
}
