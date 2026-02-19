package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeFlopSubsystem;

public class IntakeDeploy extends Command {

    private final IntakeFlopSubsystem flop;

    public IntakeDeploy(IntakeFlopSubsystem flop) {
        this.flop = flop;
        addRequirements(flop);
    }

    @Override
    public void initialize() {
        flop.deploy();   // send PID setpoint once
    }
@Override
    public void end(boolean interrupted) {
        flop.deploy();     // optional but recommended
    }
    
    @Override
    public boolean isFinished() {
        return flop.isDeployed();
    }

}
