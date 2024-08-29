package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.ShooterSubsystem;

public class ShooterFullSpeed extends Command{
    ShooterSubsystem subsystem;
    public ShooterFullSpeed(ShooterSubsystem subsystem){
        this.subsystem=subsystem;
    }
    @Override
    public void execute(){
        subsystem.shooterSpeedSpeaker();
    }
}
