package frc.robot.shooter;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.intake.IntakeCommands;
import frc.robot.intake.IntakeSubsystem;

public class ShootCommand extends Command{

    private final IntakeSubsystem intakeSubsystem;
    private final ShooterSubsystem shooter;
    private final Timer timer = new Timer();

    public ShootCommand(IntakeSubsystem intakeSubsystem, ShooterSubsystem shooter){
        this.intakeSubsystem = intakeSubsystem; 
        this.shooter=shooter;
        addRequirements();
    }

    @Override 
    public void initialize(){
        timer.start();
        if (!intakeSubsystem.hasNote()) cancel();
        intakeSubsystem.passthrough();
    }

    @Override 
    public boolean isFinished(){ 
        return !intakeSubsystem.hasNote() && timer.hasElapsed(3);
    }

    @Override
    public void end(boolean interrupted){
        CommandScheduler.getInstance().schedule(IntakeCommands.STOW.get());
    }
}