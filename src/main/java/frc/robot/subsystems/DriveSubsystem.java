package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
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

    DifferentialDriveOdometry odometry;
  

    MotorControllerGroup motorControllerGroupLeft;
    MotorControllerGroup motorControllerGroupRight;
    DifferentialDrive diffDrive;

    Field2d field = new Field2d();


    TalonSRX leftLeader;
    TalonSRX leftMotor;

    TalonSRX rightLeader;
    TalonSRX rightMotor;

    // Simulation specific items 
    DifferentialDrivetrainSim diffDriveSim;
    EncoderSim leftEncoderSim;
    EncoderSim rightEncoderSim;

    // Limiter used to limit how fast velocity is allowed to change
    final SlewRateLimiter limiter = new SlewRateLimiter(1.5);

    public DriveSubsystem()
    {
        init();
    }

    public void init() {
        // TODO: renumber these motor controllers with the correct ID's
        leftLeader = new TalonSRX(3);
        leftMotor = new TalonSRX(4);
        leftMotor.follow(leftLeader);
        
        rightLeader = new TalonSRX(0);
        rightMotor = new TalonSRX(1);
        rightMotor.follow(rightLeader);

        rightLeader.setInverted(true);
        rightMotor.setInverted(true);

        //diffDrive = new DifferentialDrive(motorControllerGroupRight, motorControllerGroupLeft);


        // Make sure encoders are reset to 0
        resetEncoders();


    }

    @Override
    public void periodic() {
        

        // TANK DRIVE CODE 
        /*
         * motorFrontLeft.set(ControlMode.PercentOutput, leftY);
         * motorFrontRight.set(ControlMode.PercentOutput, rightY);
         */
    }

    @Override
    public void simulationPeriodic() {
        var left = motorControllerGroupLeft.get();
        var right = motorControllerGroupRight.get();

        SmartDashboard.putNumber("left", left * RobotController.getBatteryVoltage());
        SmartDashboard.putNumber("right", right * RobotController.getBatteryVoltage());

        diffDriveSim.setInputs(
            left * RobotController.getBatteryVoltage(), 
            right * RobotController.getBatteryVoltage());
        
        
        diffDriveSim.update(0.020);
        /* 
        leftEncoderSim.setDistance(diffDriveSim.getLeftPositionMeters());
        leftEncoderSim.setRate(diffDriveSim.getLeftVelocityMetersPerSecond());
        rightEncoderSim.setDistance(diffDriveSim.getRightPositionMeters());
        rightEncoderSim.setRate(diffDriveSim.getRightVelocityMetersPerSecond());
        */
        
    }

    public void drive(double leftOutput, double rightOutput) {
        diffDrive.tankDrive(leftOutput, rightOutput);
    }

    public void tankDrive(double left, double right) {
        leftLeader.set(ControlMode.PercentOutput, left);
        rightLeader.set(ControlMode.PercentOutput, right);
    }

    public void arcadeDrive (double rotate, double drive){
        double maximum;
        double total;
        double difference;

        maximum = Math.max(Math.abs(drive), Math.abs(rotate));
        total = drive + rotate;
        difference = drive - rotate;

        if (drive >= 0){
            if (rotate >= 0){
                leftLeader.set(ControlMode.PercentOutput, maximum);
                rightLeader.set(ControlMode.PercentOutput, difference);
            }else{
                leftLeader.set(ControlMode.PercentOutput, total);
                rightLeader.set(ControlMode.PercentOutput, maximum);
            }
         }else{
            if (rotate >= 0){
                leftLeader.set(ControlMode.PercentOutput, total);
                rightLeader.set(ControlMode.PercentOutput, -maximum);

            }else{
                leftLeader.set(ControlMode.PercentOutput, -maximum);
                rightLeader.set(ControlMode.PercentOutput, difference);
            }
         }




    }
    public void resetEncoders()
    {
    }

    public Pose2d getPose()
    {
        return odometry.getPoseMeters();
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

    public void setBrakeMode(boolean isBrake)
    {
        if (isBrake)
        {
            //leftLeader.setIdleMode(IdleMode.kBrake);
        }
        else {
            //leftLeader.setIdleMode(IdleMode.kBrake);
        }
    }
}
