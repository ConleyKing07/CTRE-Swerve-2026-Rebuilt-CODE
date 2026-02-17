package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeFlopSubsystem;

public class IntakeRetract extends Command {

    private final IntakeFlopSubsystem flop;

    public IntakeRetract(IntakeFlopSubsystem flop) {
        this.flop = flop;
        addRequirements(flop);
    }

    @Override
    public void initialize() {
        flop.stow();
    }

    @Override
    public boolean isFinished() {
        return flop.isStowed();   // add this helper like we talked about
    }

    @Override
    public void end(boolean interrupted) {
        flop.stop();
    }
}
