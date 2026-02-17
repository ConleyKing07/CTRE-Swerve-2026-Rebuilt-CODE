package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.HopperSubsystem;

public class FireCommand extends Command {

    private final HopperSubsystem hopper;
   

    public FireCommand(
            HopperSubsystem hopper){

        this.hopper=hopper;

        addRequirements(hopper);
    }

    @Override
    public void execute(){

    
            hopper.feed();

    }

    @Override
    public void end(boolean interrupted){
        hopper.stop();
    }

    @Override
    public boolean isFinished(){
        return false;
    }
}
