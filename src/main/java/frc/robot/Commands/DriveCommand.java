package frc.robot.Commands;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.TankSubsystem;



public class DriveCommand extends Command{
    private CommandXboxController controller;
    private TankSubsystem tank;

    public DriveCommand(CommandXboxController controller, TankSubsystem tank){
        this.controller=controller;
        this.tank=tank;
    }

    @Override
    public void execute(){
        tank.driveTeleOp(controller.getLeftY(), controller.getRightY());
    }
    @Override
    public boolean isFinished(){
        return true;
    }
}