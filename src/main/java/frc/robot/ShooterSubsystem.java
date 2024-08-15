package frc.robot;

import com.ctre.phoenix.motorcontrol.VictorSPXControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
public class ShooterSubsystem extends SubsystemBase{
    private VictorSPX shooterLeft=new VictorSPX(7);
    private VictorSPX shooterRight=new VictorSPX(8);

    public void init(){
        shooterRight.setInverted(true);
    }

    public void shooterSpeedSpeaker(){
        shooterRight.set(VictorSPXControlMode.PercentOutput, 1);
        shooterLeft.set(VictorSPXControlMode.PercentOutput, 1);
    }

    public void shooterSpeedAmp(){
        shooterRight.set(VictorSPXControlMode.PercentOutput, 0.5);
        shooterLeft.set(VictorSPXControlMode.PercentOutput, 0.5);
    }
}
