// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.*;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.GlobalVariables;
import frc.robot.Constants.kElevator;

public class sysElevator extends SubsystemBase {
  /** Creates a new sysElevator. */
  private SparkMax mtrLeftEle = new SparkMax(kElevator.LEFT_CANID, MotorType.kBrushless);
  private SparkMax mtrRightEle = new SparkMax(kElevator.RIGHT_CANID, MotorType.kBrushless);

  private SparkMaxConfig cfgLeftEle = new SparkMaxConfig();
  private SparkMaxConfig cfgRightEle = new SparkMaxConfig();

  private SparkClosedLoopController ctrEle;
  private RelativeEncoder encEle;

  private double Pos;
  public boolean ele_sw_status;

  private DigitalInput ele_sw = new DigitalInput(1);
  
  public sysElevator() {

    cfgLeftEle
      .inverted(true)
      .idleMode(IdleMode.kBrake)
      .smartCurrentLimit(kElevator.CURRENT_LIMIT);
    cfgLeftEle.encoder
      .positionConversionFactor(kElevator.MTR_TO_IN)
      .velocityConversionFactor(kElevator.MTR_TO_IN);
    cfgLeftEle.closedLoop
      .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
      .pid(kElevator.KP, kElevator.KI, kElevator.KD);
    //cfgLeftEle.closedLoop.maxMotion
    //  .maxVelocity(kElevator.MAX_SPEED.in(InchesPerSecond))
    //  .maxAcceleration(kElevator.MAX_ACCEL.in(FeetPerSecondPerSecond)/12);

    cfgRightEle
      .inverted(false)
      .idleMode(IdleMode.kBrake)
      .smartCurrentLimit(kElevator.CURRENT_LIMIT);
    cfgRightEle.encoder
      .positionConversionFactor(kElevator.MTR_TO_IN)
      .velocityConversionFactor(kElevator.MTR_TO_IN);
    cfgRightEle.closedLoop
      .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
      .pid(kElevator.KP, kElevator.KI, kElevator.KD);


    cfgRightEle.follow(mtrLeftEle, true);

    mtrLeftEle.configure(cfgLeftEle, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    mtrRightEle.configure(cfgRightEle, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    ctrEle = mtrLeftEle.getClosedLoopController();
    encEle = mtrLeftEle.getEncoder();
    encEle.setPosition(0);

    Pos = getPosition();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    Pos = getPosition();

    if (ele_sw_status && !ele_sw.get() && (encEle.getVelocity() < 0)){  //On false & Elevator traveling down
      resetTO(1.5); 
    }
    ele_sw_status = ele_sw.get(); // Update previous loop value
    
    
    SmartDashboard.putBoolean("Elevator Switch", ele_sw_status);
    SmartDashboard.putNumber("Elevator Position", Pos);
    //SmartDashboard.putNumber("elevator output", mtrLeftEle.getOutputCurrent());
   
    
  }

  public void setDC(double pwr){
    mtrLeftEle.set(pwr);
  }

  public void setPosition(double inches){
    ctrEle.setReference(inches, ControlType.kPosition);
  }

  public double getPosition(){
    return encEle.getPosition();
  }
  public double getArmMax(){
    return GlobalVariables.Arm_Max;
  }

  public double getArmMin(){
    return GlobalVariables.Arm_Min;
  }

  public void reset(){
    encEle.setPosition(0);
  }

  public void resetTO(double pos){
    encEle.setPosition(pos);
  }
}
