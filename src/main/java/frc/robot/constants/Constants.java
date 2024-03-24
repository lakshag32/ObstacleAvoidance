package frc.robot.constants;

import edu.wpi.first.math.util.Units;

public class Constants {
    /** The REV Duty Cycle encoder DIO channel */
    public static final int leftMotorID = 11;
    public static final int rightMotorID = 6;
    public static final double TRACK_WIDTH = Units.inchesToMeters(25.75);
    public static final double DISTANCE_PER_ENCODER_PULSE = 1; 
    public static final double VELOCITY_CONVERSION_FACTOR = 1; //RPM to m/s 
    public static final double kP = 0.1; //RPM to m/s 
    public static final double kI = 0; //RPM to m/s 
    public static final double kD = 1; //RPM to m/s 






}
