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
        drive.turnAndDrive(0.3, -10);
        // drive.drive(0.1, 10.0);
    }

    @Override
    public boolean isFinished() {
        return false; 
    }

    @Override
    public void end(boolean interrupted) {
    }
}
