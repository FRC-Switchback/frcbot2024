package frc.robot.chassis;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;


public class DriveCommand extends Command{
    private final CommandXboxController controller;
    private final TankSubsystem tank;

    public DriveCommand(CommandXboxController controller, TankSubsystem tank){
        this.controller=controller;
        this.tank=tank;
    }

    @Override
    public void execute(){
        tank.drive(controller.getLeftY(), controller.getRightY());
    }
    @Override
    public boolean isFinished(){
        return true;
    }
}