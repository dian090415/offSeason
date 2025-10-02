package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.NewDrive.drive;

public class AutoCmd extends Command{
    private final drive swerve;
    public AutoCmd(drive swerve){
        this.swerve = swerve;
    }
}
