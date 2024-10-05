package frc.robot.intake;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.shooter.ShooterSubsystem;

import java.util.function.Supplier;

public class IntakeCommands {
    private static final IntakeSubsystem intake = IntakeSubsystem.getInstance();
    private static final ShooterSubsystem shooter = ShooterSubsystem.getInstance();

    public static final Supplier<Command> STOW = () -> Commands.run(intake::stow, intake)
            .andThen(Commands.run(shooter::shooterSpeedAmp));
    public static final Supplier<Command> INTAKE = () -> Commands.run(intake::deployAndIntake, intake);
            //.until(() -> !intake.hasNote());
    public static final Supplier<Command> AUTO_INTAKE = () -> Commands.run(intake::deployAndIntake, intake);
    public static final Supplier<Command> EJECT = () -> Commands.run(intake::outtake);
}