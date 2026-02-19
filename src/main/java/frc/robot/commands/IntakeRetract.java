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
        flop.stow();   // send PID setpoint once
    }
@Override
    public void end(boolean interrupted) {
        flop.stow();     // optional but recommended
    }
    
    @Override
    public boolean isFinished() {
        return flop.isDeployed();
    }

}
