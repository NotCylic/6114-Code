// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkMaxAbsoluteEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxAbsoluteEncoder.Type;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class Arm extends SubsystemBase {
  private CANSparkMax armLift;
  public SparkMaxPIDController arm_pidController;
  public double kP, kI, kD, kIz, kFF, kMaxOutput, kMinOutput;
  
  PIDController pid = new PIDController(1.8,0, 0);
  /**
   * An alternate encoder object is constructed using the GetAlternateEncoder() 
   * method on an existing CANSparkMax object. If using a REV Through Bore 
   * Encoder, the type should be set to quadrature and the counts per 
   * revolution set to 8192
   */
  private SparkMaxAbsoluteEncoder arm_absoluteEncoder;


  /** Creates a new Arm. */
  public Arm() {
   // initialize motor
   armLift = new CANSparkMax(12, MotorType.kBrushless);
   armLift.setInverted(false);
   arm_absoluteEncoder = armLift.getAbsoluteEncoder(Type.kDutyCycle);
   arm_absoluteEncoder.setInverted(true);


     /**
     * In order to use PID functionality for a controller, a SparkMaxPIDController object
     * is constructed by calling the getPIDController() method on an existing
     * CANSparkMax object
     */
    arm_pidController = armLift.getPIDController();
  
    /**
     * By default, the PID controller will use the Hall sensor from a NEO for its
     * feedback device. Instead, we can set the feedback device to the alternate
     * encoder object
     */
    // arm_pidController.setFeedbackDevice(arm_absoluteEncoder);

    
    /**
     * From here on out, code looks exactly like running PID control with the 
     * built-in NEO encoder, but feedback will come from the alternate encoder
     */ 

    // PID coefficients
    kP = 1.4; 
    kI = 0;
    kD = 0; 
    kIz = 0; 
    kFF = 0; 
    kMaxOutput = 1; 
    kMinOutput = -1;

    // set PID coefficients
    arm_pidController.setP(kP);
    arm_pidController.setI(kI);
    arm_pidController.setD(kD);
    arm_pidController.setIZone(kIz);
    arm_pidController.setFF(kFF);
    arm_pidController.setOutputRange(kMinOutput, kMaxOutput);

    // display PID coefficients on SmartDashboard
    SmartDashboard.putNumber("P Gain", kP);
    SmartDashboard.putNumber("I Gain", kI);
    SmartDashboard.putNumber("D Gain", kD);
    SmartDashboard.putNumber("I Zone", kIz);
    SmartDashboard.putNumber("Feed Forward", kFF);
    SmartDashboard.putNumber("Max Output", kMaxOutput);
    SmartDashboard.putNumber("Min Output", kMinOutput);
    SmartDashboard.putNumber("Set Rotations", 0.001);
    pid.setSetpoint(0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  // read PID coefficients from SmartDashboard
  double p = SmartDashboard.getNumber("P Gain", 0);
  double i = SmartDashboard.getNumber("I Gain", 0);
  double d = SmartDashboard.getNumber("D Gain", 0);
  double iz = SmartDashboard.getNumber("I Zone", 0);
  double ff = SmartDashboard.getNumber("Feed Forward", 0);
  double max = SmartDashboard.getNumber("Max Output", 0);
  double min = SmartDashboard.getNumber("Min Output", 0);
  double rotations = SmartDashboard.getNumber("Set Rotations", 0.001);

  // if PID coefficients on SmartDashboard have changed, write new values to controller
  if((p != kP)) { arm_pidController.setP(p); kP = p; }
  if((i != kI)) { arm_pidController.setI(i); kI = i; }
  if((d != kD)) { arm_pidController.setD(d); kD = d; }
  if((iz != kIz)) { arm_pidController.setIZone(iz); kIz = iz; }
  if((ff != kFF)) { arm_pidController.setFF(ff); kFF = ff; }
  if((max != kMaxOutput) || (min != kMinOutput)) { 
    arm_pidController.setOutputRange(min, max); 
    kMinOutput = min; kMaxOutput = max;}

  double position = arm_absoluteEncoder.getPosition();
  if (position > 0.5) position -= 1;

  double power = pid.calculate(position);
  if (power > 1) power = 1;
  if (power < -1) power = -1;
  armLift.set(power);
  //armLift.set(0.4);
  

  SmartDashboard.putNumber("lift power", power);

  /**
   * 
   * PIDController objects are commanded to a set point using the 
   * SetReference() method.
   * 
   * The first parameter is the value of the set point, whose units vary
   * depending on the control type set in the second parameter.
   * 
   * The second parameter is the control type can be set to one of four 
   * parameters:
   *  com.revrobotics.CANSparkMax.ControlType.kDutyCycle
   *  com.revrobotics.CANSparkMax.ControlType.kPosition
   *  com.revrobotics.CANSparkMax.ControlType.kVelocity
   *  com.revrobotics.CANSparkMax.ControlType.kVoltage
   */
 
   //arm_pidController.setReference(rotations, CANSparkMax.ControlType.kPosition);

  SmartDashboard.putNumber("SetPoint", rotations);
  SmartDashboard.putNumber("ProcessVariable", arm_absoluteEncoder.getPosition());
  
  }

  public void armDown() {
    //arm_pidController.setReference(0.001, CANSparkMax.ControlType.kPosition);
    pid.setSetpoint(0);
  }

  public void armUp1(){
    //arm_pidController.setReference(0.23, CANSparkMax.ControlType.kPosition);
    pid.setSetpoint(0.23);
  }

  public void armUp2(){
    //arm_pidController.setReference(0.27, CANSparkMax.ControlType.kPosition);
    pid.setSetpoint(0.28);
  }

  public void armTravel(){
    pid.setSetpoint(0.05);
  }

  public void armPick(){
    pid.setSetpoint(.25);
  }
}
