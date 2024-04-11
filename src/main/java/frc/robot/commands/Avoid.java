package frc.robot.commands;

import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Drivetrain;

public class Avoid extends Command {
    public final NetworkTableInstance m_NetworkTableInstance;
    
    public final DoublePublisher xPub;
    public final DoublePublisher yPub;
    public final DoubleSubscriber rotationSub;

    public final Drivetrain m_drive;
    double angle; 


    public Avoid(Drivetrain drive) {        
        m_NetworkTableInstance = NetworkTableInstance.getDefault();

        NetworkTable table = m_NetworkTableInstance.getTable("RobotPosition");

        xPub = table.getDoubleTopic("x").publish();
        yPub = table.getDoubleTopic("y").publish();
        rotationSub = table.getDoubleTopic("robotAngle").subscribe(0.0);

        m_drive = drive; 
        addRequirements(m_drive);
    }

    @Override
    public void initialize() {

    }
    @Override
    public void execute() {

        double poseX = m_drive.getEstimatedX();
        double poseY = -m_drive.getEstimatedY();
        System.out.println("poseX: " + poseX); 
        System.out.println("poseY: " + poseY); 
        
        angle = -rotationSub.get(); 

        System.out.println("angle: "+ angle); 
        
        xPub.set(poseX);
        yPub.set(poseY);

        if(m_drive.turn(10)){
            m_drive.drive_straight(0.3);
        }

    }

    @Override
    public boolean isFinished() {
        return false; 
    }

    @Override
    public void end(boolean interrupted) {
    }
}
