package frc.robot.intake;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.VictorSPXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Voltage;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.RobotMap;

import static edu.wpi.first.units.Units.*;
import static frc.robot.intake.IntakeConstants.*;

public class IntakeSubsystem extends SubsystemBase{
    private static final IntakeSubsystem INSTANCE = new IntakeSubsystem();

    public static IntakeSubsystem getInstance(){
        return INSTANCE;
    }

    private IntakeSubsystem() {

    }

    private final Encoder intakeEncoder = new Encoder(RobotMap.INTAKE_ENCODER[0], RobotMap.INTAKE_ENCODER[1]);
    private final TalonSRX intakeMotor = new TalonSRX(RobotMap.INTAKE_MOTOR);
    //private final VictorSPX passthroughMotor = new VictorSPX(RobotMap.PASSTHROUGH_MOTOR);
    private final TalonSRX intakeAcuator = new TalonSRX(RobotMap.INTAKE_ACTUATOR);
    private final DigitalInput sensor = new DigitalInput(RobotMap.INTAKE_BEAM_BRAKE);
    private final PIDController pid = new PIDController(ACTUATOR_PID[0], ACTUATOR_PID[1], ACTUATOR_PID[2]);// tune this irl
    private double lastPosition = 0;
    private double position = 0;
   
    public void init(){
        intakeEncoder.reset();
        intakeAcuator.setNeutralMode(NeutralMode.Brake);
        intakeEncoder.setDistancePerPulse(1.0/2048.0);
        pid.setTolerance(SETPOINT_TOLERANCE);
        pid.setSetpoint(STOW_SETPOINT);
    }

    public void deployAndIntake(){
        deploy();
        intake();
    }

    public void passthrough() {
        //passthroughMotor.set(VictorSPXControlMode.PercentOutput, 1);
        intakeMotor.set(ControlMode.PercentOutput, 1);
    }

    public void intake(){
        intakeMotor.set(ControlMode.PercentOutput, 0.5);
        //passthroughMotor.set(VictorSPXControlMode.PercentOutput, 1);
    }

    public void deploy() {
        pid.setSetpoint(DOWN_SETPOINT);
    }

    public void stow(){
        intakeMotor.set(ControlMode.PercentOutput, 0);
        pid.setSetpoint(STOW_SETPOINT);
    }

    public void outtake(){
        intakeMotor.set(ControlMode.PercentOutput, -1);
        //passthroughMotor.set(VictorSPXControlMode.PercentOutput, -1);
    }

    public void stopSpinning(){
        intakeMotor.set(ControlMode.PercentOutput, 0);
    }
    
    public void runPid(){
        intakeAcuator.set(ControlMode.PercentOutput, pid.calculate(intakeEncoder.getDistance()));
    }

    public void getActuatorVelocity() {

    }

    public boolean hasNote(){
        return sensor.get();
    }

    @Override
    public void periodic() {
        runPid();
        lastPosition = position;
        position = intakeEncoder.getDistance();
        SmartDashboard.putNumber("Collector Position:", position);
        SmartDashboard.putNumber("Intake Actuator Bus Voltage:", intakeAcuator.getBusVoltage());
        SmartDashboard.putNumber("Intake Actuator Voltage:", intakeAcuator.getMotorOutputVoltage());
        SmartDashboard.putNumber("Collector Setpoint", pid.getSetpoint());
    }

    public SysIdRoutine getsysIdRoutine() {
        return new SysIdRoutine(
                new SysIdRoutine.Config(
                        Volts.of(2).per(Second),  //Quasistatic Ramp Rate
                        Volts.of(2), // Dynamic voltage
                        null,     // Default timeout is acceptable
                        null),
                new SysIdRoutine.Mechanism(
                        (Measure<Voltage> volts) -> intakeAcuator.set(ControlMode.PercentOutput, volts.magnitude() / intakeAcuator.getBusVoltage()),
                        log -> log.motor("motor")
                                .voltage(Volts.of(intakeAcuator.getMotorOutputVoltage()))
                                .angularPosition(Rotations.of(intakeEncoder.getDistance()))
                                .angularVelocity(RotationsPerSecond.of((position - lastPosition) / 0.02)),
                        this));
    }
}
