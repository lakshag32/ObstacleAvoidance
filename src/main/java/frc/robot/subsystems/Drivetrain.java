/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.Constants;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.util.ReplanningConfig;
import com.revrobotics.CANSparkLowLevel.MotorType;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Drivetrain extends SubsystemBase {

    private final CANSparkMax leftMotor;
    private final CANSparkMax rightMotor;
  
    private final RelativeEncoder leftEncoder;
    private final RelativeEncoder rightEncoder;

    private final ADXRS450_Gyro gyro;

    public final DifferentialDrive drive;

    public Field2d m_field;

    // Odometry class for tracking robot pose
    private final DifferentialDriveOdometry m_odometry;

    private final PIDController m_controller; 

    // private final SimpleMotorFeedforward m_feedforward = new SimpleMotorFeedforward(
    //     Constants.drive.KsLinear,
    //     Constants.drive.KvLinear,
    //     Constants.drive.KaLinear);

    private final DifferentialDriveKinematics m_driveKinematics = new DifferentialDriveKinematics(
        Units.inchesToMeters(25.75));


    public Drivetrain() {
        leftMotor = new CANSparkMax(Constants.leftMotorID, MotorType.kBrushless);
        rightMotor = new CANSparkMax(Constants.rightMotorID, MotorType.kBrushless);

        rightMotor.setInverted(true);

        leftEncoder = leftMotor.getEncoder(); 
        rightEncoder = rightMotor.getEncoder(); 

        // leftEncoder.setVelocityConversionFactor(Constants.VELOCITY_CONVERSION_FACTOR); 
        // rightEncoder.setVelocityConversionFactor(Constants.VELOCITY_CONVERSION_FACTOR); 

        gyro = new ADXRS450_Gyro();

        drive = new DifferentialDrive(leftMotor, rightMotor);
        drive.setSafetyEnabled(false);; 

        // Sets the distance per pulse for the encoders
        leftEncoder.setPositionConversionFactor(Constants.DISTANCE_PER_ENCODER_ROT);
        rightEncoder.setPositionConversionFactor(Constants.DISTANCE_PER_ENCODER_ROT);
        leftEncoder.setVelocityConversionFactor(Constants.VELOCITY_CONVERSION_FACTOR);
        rightEncoder.setVelocityConversionFactor(Constants.VELOCITY_CONVERSION_FACTOR); 
        
        resetEncoders();
        zeroHeading();

        m_odometry = new DifferentialDriveOdometry(gyro.getRotation2d(),leftEncoder.getPosition(),rightEncoder.getPosition()); 
    
        m_controller = new PIDController(0.025,0,0); //kp = 0.5, kd = 0.4. kp = 0.05 works great with /5 but slow. 
        m_controller.setTolerance(1);

        m_field = new Field2d(); 

        SmartDashboard.putData(m_field);

        AutoBuilder.configureRamsete(
            this::getPose, // Robot pose supplier
            this::resetPose, // Method to reset odometry (will be called if your auto has a starting pose)
            this::getCurrentSpeeds, // Current ChassisSpeeds supplier
            this::drive, // Method that will drive the robot given ChassisSpeeds
            new ReplanningConfig(), // Default path replanning config. See the API for the options here
            () -> {
            // Boolean supplier that controls when the path will be mirrored for the red alliance
            // This will flip the path being followed to the red side of the field.
            // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

            var alliance = DriverStation.getAlliance();
            if (alliance.isPresent()) {
                return alliance.get() == DriverStation.Alliance.Red;
            }
            return false;
            },
            this // Reference to this subsystem to set requirements
        );

    }

    //https://www.youtube.com/watch?v=gRbBkdinq0o
    public ChassisSpeeds getCurrentSpeeds(){
       return m_driveKinematics.toChassisSpeeds(new DifferentialDriveWheelSpeeds(leftEncoder.getVelocity(),rightEncoder.getVelocity()));
    }
    

    public DifferentialDriveKinematics getDriveKinematics() {
        return m_driveKinematics;
    }
    
    @Override
    public void periodic() {
        // Update the odometry in the periodic block
        updateOdometry();
        m_field.setRobotPose(getPose());
        // turnAndDrive(0.1, 1);
    }
    

    public void updateOdometry() {
        m_odometry.update(gyro.getRotation2d(), leftEncoder.getPosition(), rightEncoder.getPosition());
    }

    public void drive(ChassisSpeeds speeds){
        var wheelSpeeds = m_driveKinematics.toWheelSpeeds(speeds);
        setSpeeds(wheelSpeeds);
    }

    public void setSpeeds(DifferentialDriveWheelSpeeds speeds) {

        final double leftOutput =
            m_controller.calculate(leftEncoder.getVelocity(), speeds.leftMetersPerSecond);
    
        final double rightOutput =
            m_controller.calculate(rightEncoder.getVelocity(), speeds.rightMetersPerSecond);
        
        leftMotor.setVoltage(leftOutput);
        rightMotor.setVoltage(rightOutput);
      }
    

    public boolean turn(double desiredAngle){ 
        double currentRobotAngle = -gyro.getAngle(); 

        if(currentRobotAngle < desiredAngle){
          m_controller.setSetpoint(desiredAngle);
          double PIDPower = MathUtil.clamp(m_controller.calculate(currentRobotAngle),-0.2,0.2);

          if (m_controller.atSetpoint()){
            return true; 
          }
    
          rightMotor.set(PIDPower);
          leftMotor.set(-PIDPower);
        }
    
        if(currentRobotAngle > desiredAngle){    
          m_controller.setSetpoint(desiredAngle);
          double PIDPower = MathUtil.clamp(m_controller.calculate(currentRobotAngle),-0.2,0.2);

          if (m_controller.atSetpoint()){
            return true; 
          }
    
          rightMotor.set(PIDPower);
          leftMotor.set(-PIDPower);
        }

        return false; 
    }

    public void drive_straight(double power){
        // Setpoint is implicitly 0, since we don't want the heading to change
        double error = -gyro.getRate();
    
        // // Drives forward continuously at half speed, using the gyro to stabilize the heading
        // rightMotor.set(power - 0.0025 * error); 
        // leftMotor.set(power + 0.0025 * error); 
        System.out.println(leftEncoder.getPosition());
        drive.tankDrive(power + 0.005 * error, power - 0.005 * error);
    }

    /**
     * Returns the currently-estimated pose of the robot.
     *
     * @return The pose.
     */
    public Pose2d getPose() {
        return m_odometry.getPoseMeters();
    }

    public void resetPose(Pose2d pose){
        resetEncoders();
        m_odometry.resetPosition(gyro.getRotation2d(),leftEncoder.getPosition(),rightEncoder.getPosition(), pose);
    }

    /**
     * Returns the currently-estimated x pose of the robot.
     *
     * @return The x of the pose in meters.
     */
    public double getEstimatedX() {
        return m_odometry.getPoseMeters().getX();
    }

    /**
     * Returns the currently-estimated y pose of the robot.
     *
     * @return The y of the pose in meters.
     */
    public double getEstimatedY() {
        return m_odometry.getPoseMeters().getY();
    }

    public double getHeading() {
        return gyro.getRotation2d().getDegrees();
    }

    public double getPoseX() {
        return getPose().getX();
    }

    public double getPoseY() {
        return getPose().getY();
    }

    public void resetEncoders() {
        //TODO: FLAG
        leftEncoder.setPosition(0);
        rightEncoder.setPosition(0);
    }

    /** Zeroes the heading of the robot. */
    public void zeroHeading() {
        gyro.reset();
    }

    /**
     * Returns the currently-estimated rotation pose of the robot.
     *
     * @return The rotation in degrees.
     */
    // public double getEstimatedDegrees() {
    //     return m_odometry.getPoseMeters().getRotation().getDegrees();
    // }

    // public DifferentialDriveWheelSpeeds getWheelSpeeds() {
    //     //TODO: Ensure that the leftEncoder values are in meters per second
    //     return new DifferentialDriveWheelSpeeds(leftEncoder.getVelocity(), rightEncoder.getVelocity());
    // }

    /**
     * Controls the left and right sides of the drive directly with voltages.
     *
     * @param leftVolts  the commanded left output
     * @param rightVolts the commanded right output
     */
    // public void tankDriveVolts(double leftVolts, double rightVolts) {
    //     var batteryVoltage = RobotController.getBatteryVoltage();
    //     if (Math.max(Math.abs(leftVolts), Math.abs(rightVolts)) > batteryVoltage) {
    //     leftVolts *= batteryVoltage / Constants.kMaxVoltage;
    //     rightVolts *= batteryVoltage / Constants.kMaxVoltage;
    //     }
    //     leftMotor.setVoltage(leftVolts);
    //     rightMotor.setVoltage(rightVolts);
    //     m_dDrive.feed();
    // }

    // /**
    //  * Sets the desired wheel speeds.
    //  *
    //  * @param speeds The desired wheel speeds.
    //  */
    // public void setSpeeds(DifferentialDriveWheelSpeeds speeds) {
    //     final double leftFeedforward = m_feedforward.calculate(speeds.leftMetersPerSecond);
    //     final double rightFeedforward = m_feedforward.calculate(speeds.rightMetersPerSecond);

    //     final double leftOutput = m_leftVelocityPID.calculate(leftEncoder.getVelocity(), speeds.leftMetersPerSecond);
    //     final double rightOutput = m_rightVelocityPID.calculate(rightEncoder.getVelocity(), speeds.rightMetersPerSecond);

    //     tankDriveVolts(leftOutput + leftFeedforward, rightOutput + rightFeedforward);
    // }

    /**
     * Sets the max output of the drive. Useful for scaling the drive to drive more
     * slowly.
     *
     * @param maxOutput the maximum output to which the drive will be constrained
     */
    // public void setMaxOutput(double maxOutput) {
    //     m_dDrive.setMaxOutput(maxOutput);
    // }

    /**
     * Returns the heading of the robot.
     *
     * @return the robot's heading in degrees, from -180 to 180
     */

    // public void driveStraight(){
    //     double robotCurrentAngle = getHeading(); 

    // }

    /**
     * Drives the robot with the given linear velocity and angular velocity.
     *
     * @param xSpeed Linear velocity in m/s.
     * @param rot    Angular velocity in rad/s.
     */
    // public void feedForwardDrive(double xSpeed, double rot) {
    //     var wheelSpeeds = m_driveKinematics.toWheelSpeeds(new ChassisSpeeds(xSpeed, 0.0, rot));
    //     setSpeeds(wheelSpeeds);
    //     //m_dDrive.feed();
    // }

    // public SimpleMotorFeedforward getFeedforward() {
    //     return m_feedforward;
    // }

    // //Velocity PID for teleop
    // public PIDController getLeftVelocityPID() {
    //     return m_leftVelocityPID;
    // }
    // public PIDController getRightVelocityPID() {
    //     return m_rightVelocityPID;
    // }

}