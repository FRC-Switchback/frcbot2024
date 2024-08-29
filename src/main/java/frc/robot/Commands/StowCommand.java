package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.IntakeSubsystem;

public class StowCommand extends Command{
    IntakeSubsystem subsystem;
    public StowCommand(IntakeSubsystem subsystem){
        this.subsystem=subsystem;
    }
    @Override
    public void execute(){
        subsystem.stow();
    }
}
