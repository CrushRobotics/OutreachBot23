package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.interfaces.Gyro;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim;
import edu.wpi.first.wpilibj.simulation.EncoderSim;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Robot;


public class DriveSubsystem extends SubsystemBase {
    double WHEEL_DIAMETER = Units.inchesToMeters(5); 
    double WHEEL_CIRCUMFERENCE = WHEEL_DIAMETER * Math.PI;
    double WHEEL_GEAR_RATIO = .75;
    
    WPI_TalonSRX leftLeader;
    WPI_TalonSRX rightLeader;
    
    MotorControllerGroup motorControllerGroupLeft;
    MotorControllerGroup motorControllerGroupRight;
    DifferentialDrive diffDrive;


    public void init() {
        // TODO: renumber these motor controllers with the correct ID's
        leftLeader = new WPI_TalonSRX(1);
        motorControllerGroupLeft = new MotorControllerGroup(
        leftLeader, 
            new WPI_TalonSRX(3));
        
        rightLeader = new WPI_TalonSRX(2);
        motorControllerGroupRight = new MotorControllerGroup(
            rightLeader,
            new WPI_TalonSRX(4));

        motorControllerGroupRight.setInverted(true);

        diffDrive = new DifferentialDrive(motorControllerGroupRight, motorControllerGroupLeft);


    }

    @Override
    public void periodic() {
    

    }


    public void drive(double leftOutput, double rightOutput) {
        diffDrive.tankDrive(leftOutput, rightOutput);
    }
    
    public void arcadeDrive(double fwd, double rot)
    {
        diffDrive.arcadeDrive(fwd * dampenFactor, rot);
        //diffDrive.arcadeDrive(limiter.calculate(fwd), rot);
    }

    public void tankDriveVolts(double left, double right)
    {
        motorControllerGroupLeft.setVoltage(left);
        motorControllerGroupRight.setVoltage(right);
        diffDrive.feed();
    }

    public void setMaxOutput(double maxOutput)
    {
        diffDrive.setMaxOutput(maxOutput);
    }

    public void setTurnSpeed(double turnSpeed)
    {
        diffDrive.curvatureDrive(0, turnSpeed, true);
    }
}
