// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;


import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase {
  PWMSparkMax leftIntake;
  PWMSparkMax rightIntake;
 
  /** Creates a new Intake. */
  public Intake() {
    leftIntake = new PWMSparkMax(0);
    leftIntake.setInverted(false);
    rightIntake = new PWMSparkMax(1);
    rightIntake.setInverted(true);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
  

  public void intakeCube(double speed){
    leftIntake.set(speed);
    rightIntake.set(speed);
  }

  public void intakeCone(double speed){
    leftIntake.set(speed);
    rightIntake.set(speed);
  }

  public void dropObject(double speed){
    leftIntake.set(speed);
    rightIntake.set(speed);
  }



  public void idle(double speed){
    leftIntake.set(speed);
    rightIntake.set(speed);
  }

  public void stop(){
    leftIntake.set(0);
    rightIntake.set(0);
  }

}
