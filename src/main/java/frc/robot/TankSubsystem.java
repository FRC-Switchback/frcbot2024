package frc.robot;
//TODO: add pathplanner
//TODO:auto stuff
import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class TankSubsystem extends SubsystemBase{
     private final TalonSRX leftmotor=new TalonSRX(3);
  private final TalonSRX leftmotorfollow=new TalonSRX(4);
  private final TalonSRX rightmotor=new TalonSRX(5);
  private final TalonSRX rightmotorfollow=new TalonSRX(6);// can ids of the talons

  public void init(){
    rightmotor.setInverted(true);
    rightmotorfollow.setInverted(true);
    rightmotorfollow.follow(rightmotor);//inverting properly(change if robot moves backwords)
    leftmotorfollow.follow(leftmotor);
  }

  public void drive(double leftStick, double rightStick){
     leftmotor.set(TalonSRXControlMode.PercentOutput,leftStick);//run motors
     rightmotor.set(TalonSRXControlMode.PercentOutput,rightStick);
  }
}
