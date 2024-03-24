package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Drivetrain;

public class Avoid extends Command {
    public final Drivetrain drive;

    public Avoid(Drivetrain drive) {
        this.drive = drive; 
        addRequirements(drive);
    }

    @Override
    public void initialize() {

    }
    @Override
    public void execute() {
        if(drive.turn(5)){
            drive.drive_straight(0.3);
        }
        // // drive.drive(0.1, 10.0);
        // drive.drive_straight(0.1);
    }

    @Override
    public boolean isFinished() {
        return false; 
    }

    @Override
    public void end(boolean interrupted) {
    }
}
