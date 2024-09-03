package frc.robot;
//TODO: add pathplanner
//TODO:auto stuff
import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import com.ctre.phoenix6.hardware.CANcoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class TankSubsystem extends SubsystemBase{
  private static final TankSubsystem INSTANCE = new TankSubsystem();

    public static TankSubsystem getInstance(){
        return INSTANCE;
    }

    private TankSubsystem() {

    }
  private final TalonSRX leftmotor=new TalonSRX(3);
  private final TalonSRX leftmotorfollow=new TalonSRX(4);
  private final TalonSRX rightmotor=new TalonSRX(5);
  private final TalonSRX rightmotorfollow=new TalonSRX(6);// can ids of the talons
    private final CANcoder leftEncoder = new CANcoder(11);
    private final CANcoder rightEncoder = new CANcoder(12);

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

  public double getLeftDrivePosition(){
    return leftEncoder.getPosition().getValueAsDouble();
  }

  public double getRightDrivePosition(){
    return rightEncoder.getPosition().getValueAsDouble();
  }

  public double[] getDrivePosition() {
      return new double[]{getLeftDrivePosition(), getRightDrivePosition()};
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Left Drive Position", getLeftDrivePosition());
    SmartDashboard.putNumber("Right Drive Position", getRightDrivePosition());
  }
}
