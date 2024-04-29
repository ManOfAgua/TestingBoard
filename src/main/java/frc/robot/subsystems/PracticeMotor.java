// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.ResourceBundle.Control;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import com.ctre.phoenix6.configs.HardwareLimitSwitchConfigs;
import com.ctre.phoenix6.configs.SoftwareLimitSwitchConfigs;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.ControlModeValue;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.PS5Controller;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class PracticeMotor extends SubsystemBase {
  DigitalInput limitSwitch = new DigitalInput(0);
  TalonFX m_motor = new TalonFX(4);

  //Photon
  PhotonCamera camera = new PhotonCamera("PhotonCamera");
  //Attempting to get Fiducial ID
  public int DebugFiducialID()
  {
    var result = camera.getLatestResult();
    PhotonTrackedTarget target = result.getBestTarget();
    int targetID = target.getFiducialId();
    return targetID;
  }

  public PracticeMotor() {
  }
  public void raisePID(double speed){
    this.m_motor.set(speed);
  }
  public void raise(){
    if(!this.softLimit()){
      this.setMotorSpeed(0.1);
    }
    else{
      this.setMotorSpeed(0);
    }
  }

  public void raisePID(){
    if(!this.softLimit()){
    }
  }

  private void setMotorSpeed(double speed) {
    this.m_motor.set(speed);
  }

  public void stopMotor(){
    this.m_motor.set(0);
  }
  public boolean softLimit(){ //this works
    if(shooterAngle()>=15){
      return true;
    }
    else{ 
    return false;
    }
  }
  public boolean hardLimit(){ //this also works
    if(limitSwitch.get()==true){
      return false;
    }
    else{
      return true;
    }
  } 
  

  public double shooterAngle(){
  if(hardLimit()){
    m_motor.setPosition(0);
  } 
   double rotations = (m_motor.getPosition().getValueAsDouble()/2048)*360;
  return rotations;
}

  @Override
  public void periodic() {
    SmartDashboard.putBoolean("Switch", hardLimit());
    SmartDashboard.putNumber("ShooterAngle", shooterAngle());
    SmartDashboard.putNumber("Fiducial ID", DebugFiducialID() );
    shooterAngle();
  }
}
