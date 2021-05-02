package frc.robot.commands.auto;

import java.io.IOException;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.CommandGroupBase;
import frc.robot.commands.auto.paths.*;
import frc.robot.subsystems.RomiDrivetrain;

public class AutoCommand extends CommandBase{
    PathBase path1, path2, path3;
    boolean finished = false;
    public AutoCommand(RomiDrivetrain subsystem){
        //addRequirements(subsystem);
        try{
            path1 = new FirstPath(subsystem);
            path2 = new SecondPath(subsystem);
            path3 = new ThirdPath(subsystem);
        } catch (IOException e) {
            e.printStackTrace();
        }
    }
    @Override
    public void execute() {
        System.out.println(path2.finished);
        if (!path1.hasRun) path1.start();
        if (path1.finished && !path2.hasRun) path2.start();
        if (path2.finished && !path3.hasRun) path3.start();
        if (path3.finished) finished = true;
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
