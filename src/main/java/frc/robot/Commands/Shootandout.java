package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.IntakeSubsystem;

public class Shootandout extends Command{
    IntakeSubsystem subsystem;
    public Shootandout(IntakeSubsystem subsystem){
        this.subsystem=subsystem;
    }
    @Override
    public void execute(){
        subsystem.outtake();
    }
}
