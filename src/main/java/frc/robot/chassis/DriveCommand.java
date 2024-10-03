package frc.robot.chassis;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

import java.util.function.DoubleSupplier;


public class DriveCommand extends Command{
    private final TankSubsystem tank = TankSubsystem.getInstance();

    private final DoubleSupplier leftY;
    private final DoubleSupplier rightY;

    public DriveCommand(DoubleSupplier leftY, DoubleSupplier rightY){
        this.leftY = leftY;
        this.rightY = rightY;
        addRequirements(tank);
    }

    @Override
    public void execute(){
        tank.drive(leftY.getAsDouble(), rightY.getAsDouble());
        SmartDashboard.putNumber("Right Y", rightY.getAsDouble());
    }
    @Override
    public boolean isFinished(){
        return true;
    }
}