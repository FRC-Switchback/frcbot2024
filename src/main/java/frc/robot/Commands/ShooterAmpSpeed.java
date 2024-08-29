package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.ShooterSubsystem;

public class ShooterAmpSpeed extends Command{
    ShooterSubsystem subsystem;
    public ShooterAmpSpeed(ShooterSubsystem subsystem){
        this.subsystem=subsystem;
    }
    @Override
    public void execute(){
        subsystem.shooterSpeedAmp();
    }
}
