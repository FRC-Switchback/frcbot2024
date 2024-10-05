package frc.robot.shooter;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

import java.util.function.Function;
import java.util.function.Supplier;

public class ShooterCommands {
    private static final ShooterSubsystem shooter = ShooterSubsystem.getInstance();

    public static final Supplier<Command> SHOOTER_AMP_SPEED = () -> Commands.run(shooter::shooterSpeedAmp, shooter);
    public static final Supplier<Command> SHOOTER_FULL_SPEED = () -> Commands.run(shooter::shooterSpeedSpeaker, shooter);
    public static final Supplier<Command> SHOOTER_OFF = () -> Commands.runOnce(shooter::shooterSpeedOff, shooter);
}
