package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.IntakeSubsystem;

public class IntakeCommand extends Command{
    IntakeSubsystem subsystem;
    public IntakeCommand(IntakeSubsystem subsystem){
        this.subsystem=subsystem;
    }
    @Override
    public void execute(){
        subsystem.intakeDown();
    }
}
