package frc.robot;

import com.ctre.phoenix.motorcontrol.VictorSPXControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
public class IntakeSubsystem extends SubsystemBase{//MAKE SURE THE INTAKE CAN NOT GO PAST ITS 0
    //IF IT GOES PAST ITS 0 PID WILL FREAK OUT(sorry for all caps)
    private final double intakeDownSetpoint=0;//tune irl
    private final double intakeStowSetpoint=0;//tune irl

    private final Encoder intakeEncoder=new Encoder(0, 1);
    private final VictorSPX intakeMotor=new VictorSPX(9);
    private final VictorSPX intakeAcuator=new VictorSPX(10);
    private final DigitalInput sensor=new DigitalInput(2);
    private final PIDController pid=new PIDController(0, 0, 0);// tune this irl
   
    public void init(){
        intakeEncoder.reset();
        pid.setSetpoint(intakeStowSetpoint);
        pid.setTolerance(40);//tune this until all the sequential commands run
        //and dont get stuck bc its not in tolorance
    }

    public void intakeDown(){
        intakeMotor.set(VictorSPXControlMode.PercentOutput, 1);
        pid.setSetpoint(intakeDownSetpoint);
    }

    public void stow(){
        intakeMotor.set(VictorSPXControlMode.PercentOutput, 0);
        pid.setSetpoint(intakeStowSetpoint);
    }

    public void outtake(){
        intakeMotor.set(VictorSPXControlMode.PercentOutput, -1);
    }

    public void stopSpinning(){
        intakeMotor.set(VictorSPXControlMode.PercentOutput, 0);
    }
    
    public void runPid(){
        intakeAcuator.set(VictorSPXControlMode.PercentOutput, pid.calculate(intakeEncoder.getDistance()));
    }
    public boolean hasNote(){
        return sensor.get();
    }
    @Override
    public void periodic() {
      runPid();
    }

}
