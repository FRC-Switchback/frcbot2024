package frc.robot.chassis;
import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Voltage;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.RobotMap;

import static edu.wpi.first.units.Units.*;
import static edu.wpi.first.units.Units.RotationsPerSecond;

public class TankSubsystem extends SubsystemBase{
    private static final TankSubsystem INSTANCE = new TankSubsystem();

    private TankSubsystem() {
    }

    public static TankSubsystem getInstance() {
        return INSTANCE;
    }

    private final TalonSRX leftmotor=new TalonSRX(RobotMap.CHASSIS_LEFT_MOTOR);
    private final TalonSRX leftmotorfollow=new TalonSRX(RobotMap.CHASSIS_LEFT_FOLLOW_MOTOR);
    private final TalonSRX rightmotor=new TalonSRX(RobotMap.CHASSIS_RIGHT_MOTOR);
    private final TalonSRX rightmotorfollow=new TalonSRX(RobotMap.CHASSIS_RIGHT_FOLLOW_MOTOR);// can ids of the talons
    private final Encoder leftEncoder = new Encoder(RobotMap.CHASSIS_LEFT_ENCODER[0], RobotMap.CHASSIS_LEFT_ENCODER[1]);
    private final Encoder rightEncoder = new Encoder(RobotMap.CHASSIS_RIGHT_ENCODER[0], RobotMap.CHASSIS_RIGHT_ENCODER[1]);

//    private final AHRS navx = new AHRS();
//    private final DifferentialDriveOdometry odometry = new DifferentialDriveOdometry(navx.getRotation2d(),
//            getLeftDrivePosition(),
//            getRightDrivePosition());
    private final DifferentialDriveKinematics kinematics = new DifferentialDriveKinematics(TankConstants.CHASSIS_WIDTH_METERS);
    private Pose2d currentPose = new Pose2d();
    private final PIDController leftDrivePidController = new PIDController(TankConstants.DRIVE_PID.kP, TankConstants.DRIVE_PID.kI, TankConstants.DRIVE_PID.kD);
    private final PIDController rightDrivePidController = new PIDController(TankConstants.DRIVE_PID.kP, TankConstants.DRIVE_PID.kI, TankConstants.DRIVE_PID.kD);
    private double leftLastPosition = getLeftDrivePosition();
    private double rightLastPosition = getRightDrivePosition();
    private double leftPosition = getLeftDrivePosition();
    private double rightPosition = getRightDrivePosition();

    public void init(){
        rightmotor.setInverted(true);
        rightmotorfollow.setInverted(true);
        rightmotorfollow.follow(rightmotor);//inverting properly(change if robot moves backwords)
        leftmotorfollow.follow(leftmotor);
        leftEncoder.setDistancePerPulse(1.0/1024.0);
        rightEncoder.setDistancePerPulse(1.0/1024.0);
        leftDrivePidController.setSetpoint(0);
        rightDrivePidController.setSetpoint(0);
        zeroPosition();
    }

    public void drive(double leftStick, double rightStick){
        leftmotor.set(TalonSRXControlMode.PercentOutput,leftStick);//run motors
        rightmotor.set(TalonSRXControlMode.PercentOutput,rightStick);
    }

    @Override
    public void periodic() {
        leftLastPosition = leftPosition;
        rightLastPosition = rightPosition;
        leftPosition = getLeftDrivePosition();
        rightPosition = getRightDrivePosition();
//        currentPose = odometry.update(navx.getRotation2d(),
//                getLeftDrivePosition() ,
//                getRightDrivePosition());

        if (DriverStation.isAutonomous()) {
            leftmotor.set(TalonSRXControlMode.PercentOutput, leftDrivePidController.calculate(getLeftDriveVelocity()));
            rightmotor.set(TalonSRXControlMode.PercentOutput, rightDrivePidController.calculate(getRightDriveVelocity()));
        }

        SmartDashboard.putNumber("Left Pos", getLeftDrivePosition());
        SmartDashboard.putNumber("Right Pos", getRightDrivePosition());
        SmartDashboard.putNumber("Right Vel", getLeftDriveVelocity());
        SmartDashboard.putNumber("Left Vel", getLeftDriveVelocity());
    }

    public void zeroPosition() {
        leftEncoder.reset();
        rightEncoder.reset();
    }

    public double getLeftDrivePosition(){
        return leftEncoder.getDistance() * TankConstants.GEAR_RATIO * TankConstants.WHEEL_CIRCUMFERENCE_METERS;
    }

    public double getRightDrivePosition(){
        return rightEncoder.getDistance() * TankConstants.GEAR_RATIO * TankConstants.WHEEL_CIRCUMFERENCE_METERS;
    }


    //NOTE: Velocities might be a control loop behind, depending on exact call order
    public double getLeftDriveVelocity(){
        return getLeftDrivePosition() - leftLastPosition;
    }

    public double getRightDriveVelocity(){
        return getRightDrivePosition() - rightLastPosition;
    }

    public double[] getDrivePosition() {
        return new double[]{getLeftDrivePosition(), getRightDrivePosition()};
    }

    public void setPose(Pose2d pose) {
        currentPose = pose;
//        odometry.resetPosition(pose.getRotation(), getLeftDrivePosition(), getRightDrivePosition(), pose);
    }

    public Pose2d getPose() {
        return currentPose;
    }

    public ChassisSpeeds getSpeeds() {
        return kinematics.toChassisSpeeds(new DifferentialDriveWheelSpeeds(getLeftDriveVelocity(), getRightDriveVelocity()));
    }

    public void driveChassisSpeeds(ChassisSpeeds targetSpeeds) {
        DifferentialDriveWheelSpeeds wheelSpeeds = kinematics.toWheelSpeeds(targetSpeeds);

        leftDrivePidController.setSetpoint(wheelSpeeds.leftMetersPerSecond);
        rightDrivePidController.setSetpoint(wheelSpeeds.rightMetersPerSecond);
    }

    public SysIdRoutine getLeftsysIdRoutine() {
        return new SysIdRoutine(
                new SysIdRoutine.Config(
                        Volts.of(0.25).per(Second),  //Quasistatic Ramp Rate
                        Volts.of(1), // Dynamic voltage
                        null,     // Default timeout is acceptable
                        null),
                new SysIdRoutine.Mechanism(
                        (Measure<Voltage> volts) -> leftmotor.set(TalonSRXControlMode.PercentOutput.toControlMode(), volts.magnitude() / 12),
                        log -> log.motor("motor")
                                .voltage(Volts.of(leftmotor.getMotorOutputVoltage()))
                                .angularPosition(Rotations.of(leftEncoder.getDistance()))
                                .angularVelocity(RotationsPerSecond.of(getLeftDriveVelocity())),
                        this));
    }

    public SysIdRoutine getRightsysIdRoutine() {
        return new SysIdRoutine(
                new SysIdRoutine.Config(
                        Volts.of(0.25).per(Second),  //Quasistatic Ramp Rate
                        Volts.of(1), // Dynamic voltage
                        null,     // Default timeout is acceptable
                        null),
                new SysIdRoutine.Mechanism(
                        (Measure<Voltage> volts) -> rightmotor.set(TalonSRXControlMode.PercentOutput.toControlMode(), volts.magnitude() / 12),
                        log -> log.motor("motor")
                                .voltage(Volts.of(rightmotor.getMotorOutputVoltage()))
                                .angularPosition(Rotations.of(rightEncoder.getDistance()))
                                .angularVelocity(RotationsPerSecond.of(getLeftDriveVelocity())),
                        this));
    }
}