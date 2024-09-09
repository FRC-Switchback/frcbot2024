package frc.robot.chassis;
//TODO: add pathplanner
//TODO:auto stuff
import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import com.ctre.phoenix6.hardware.CANcoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap;

public class TankSubsystem extends SubsystemBase{
    private static final TankSubsystem INSTANCE = new TankSubsystem();

    private TankSubsystem() {
    }

    public static TankSubsystem getInstance() {
        return INSTANCE;
    }

    private final TalonSRX leftmotor=new TalonSRX(RobotMap.CHASSIS_LEFT_MOTOR);
    private final TalonSRX leftmotorfollow=new TalonSRX(RobotMap.CHASSIS_LEFT_FOLLOW_MOTOR);
    private final TalonSRX rightmotor=new TalonSRX(RobotMap.CHASSIS_RIGHT_ENCODER);
    private final TalonSRX rightmotorfollow=new TalonSRX(RobotMap.CHASSIS_RIGHT_FOLLOW_MOTOR);// can ids of the talons
    private final CANcoder leftEncoder = new CANcoder(RobotMap.CHASSIS_LEFT_ENCODER);
    private final CANcoder rightEncoder = new CANcoder(RobotMap.CHASSIS_RIGHT_ENCODER);

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
}
