// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class sysElevator extends SubsystemBase {
  /** Creates a new sysElevator. */
  private SparkMax mtrLeftEle = new SparkMax(0, MotorType.kBrushless);
  private SparkMax mtrRightEle = new SparkMax(0, MotorType.kBrushless);

  private SparkMaxConfig cfgLeftEle = new SparkMaxConfig();
  private SparkMaxConfig cfgRightEle = new SparkMaxConfig();

  private SparkClosedLoopController ctrEle;
  private RelativeEncoder encEle;
  
  public sysElevator() {

    cfgLeftEle.inverted(true)
      .idleMode(IdleMode.kBrake)
      .encoder.positionConversionFactor(1000).velocityConversionFactor(1000);
    cfgLeftEle.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder)
      .pid(1.0, 0, 0);

      cfgRightEle.inverted(true)
      .idleMode(IdleMode.kBrake)
      .encoder.positionConversionFactor(1000).velocityConversionFactor(1000);
    cfgRightEle.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder)
      .pid(1.0, 0, 0);


    cfgLeftEle.inverted(true)
      .idleMode(IdleMode.kBrake);
    cfgRightEle.follow(mtrLeftEle, true);

    mtrLeftEle.configure(cfgLeftEle, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    mtrRightEle.configure(cfgRightEle, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    ctrEle = mtrLeftEle.getClosedLoopController();
    encEle = mtrLeftEle.getEncoder();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
