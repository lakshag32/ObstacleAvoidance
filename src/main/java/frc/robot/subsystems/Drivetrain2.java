package frc.robot.subsystems;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Drivetrain2 extends SubsystemBase {
    CANSparkMax leftMotor;
    CANSparkMax rightMotor;
    ADXRS450_Gyro gyro = new ADXRS450_Gyro();

    double driveAngle;

    /**
     * Creates a new Swerve Style Drivetrain.
     */
    public Drivetrain2() {
        leftMotor = new CANSparkMax(11, MotorType.kBrushless);
        rightMotor = new CANSparkMax(6, MotorType.kBrushless);

        rightMotor.setInverted(true);

    }

    @Override
    public void periodic() {
        //rightMotor.set(0.1);
        leftMotor.set(0.1);
    }
    
    public void moveAtAngle() {

    }



}
