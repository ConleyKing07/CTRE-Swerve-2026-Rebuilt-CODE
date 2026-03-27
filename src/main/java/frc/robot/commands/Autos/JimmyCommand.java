package frc.robot.commands.Autos;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeFlopSubsystem;

public class JimmyCommand extends Command {

    private final IntakeFlopSubsystem intakeFlop;

    public JimmyCommand(IntakeFlopSubsystem intakeFlop) {
        this.intakeFlop = intakeFlop;

        addRequirements(intakeFlop); // IMPORTANT
    }

    @Override
    public void initialize() {
        intakeFlop.startJimmy();
    }

    @Override
    public void execute() {
        // Nothing needed — subsystem handles oscillation in periodic()
    }

    @Override
    public void end(boolean interrupted) {
        intakeFlop.stopJimmy();
    }

    @Override
    public boolean isFinished() {
        return false; // runs until canceled
    }
}