// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.PS4Controller;
import edu.wpi.first.wpilibj.TimedRobot;

/**
 * This is a demo program showing the use of the DifferentialDrive class, specifically it contains
 * the code necessary to operate a robot with tank drive.
 */
public class Robot extends TimedRobot {
  
  private double m_leftStick;
  private double m_rightStick;
  private final TalonSRX leftmotor=new TalonSRX(4);
  private final TalonSRX leftmotorfollow=new TalonSRX(5);
  private final TalonSRX rightmotor=new TalonSRX(6);
  private final TalonSRX rightmotorfollow=new TalonSRX(7);
  PS4Controller controller=new PS4Controller(0);
 

  @Override
  public void robotInit() {
    rightmotor.setInverted(true);
    rightmotorfollow.setInverted(true);
    rightmotorfollow.follow(rightmotor);
    leftmotorfollow.follow(leftmotor);
    
    
    // We need to invert one side of the drivetrain so that positive voltages
    // result in both sides moving forward. Depending on how your robot's
    // gearbox is constructed, you might have to invert the left side instead.
    

    
    m_leftStick =-controller.getLeftY();
    m_rightStick = -controller.getRightY();
  }

  @Override
  public void teleopPeriodic() {
    
    leftmotor.set(TalonSRXControlMode.PercentOutput,m_leftStick);
    rightmotor.set(TalonSRXControlMode.PercentOutput,m_rightStick);
  
  }
}
