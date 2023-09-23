package frc.robot.subsystems;

<<<<<<< HEAD
import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANSparkMax;
import com.revrobotics.REVPhysicsSim;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
=======
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
>>>>>>> 01ea2f83f98265c1c412afd23ff53a949267c9ef

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
<<<<<<< HEAD
import frc.robot.Constants.DriveConstants;
=======

>>>>>>> 01ea2f83f98265c1c412afd23ff53a949267c9ef

public class DriveSubsystem extends SubsystemBase {
    double WHEEL_DIAMETER = Units.inchesToMeters(5); 
    double WHEEL_CIRCUMFERENCE = WHEEL_DIAMETER * Math.PI;
    double WHEEL_GEAR_RATIO = .75;
<<<<<<< HEAD
    double dampenFactor = DriveConstants.dampenFactor;

    DifferentialDriveOdometry odometry;
  
    AHRS gyro;

=======
    
    WPI_TalonSRX leftLeader;
    WPI_TalonSRX rightLeader;
    
>>>>>>> 01ea2f83f98265c1c412afd23ff53a949267c9ef
    MotorControllerGroup motorControllerGroupLeft;
    MotorControllerGroup motorControllerGroupRight;
    DifferentialDrive diffDrive;

<<<<<<< HEAD
    Field2d field = new Field2d();

    RelativeEncoder leftEncoder;
    RelativeEncoder rightEncoder;

    CANSparkMax leftLeader;
    CANSparkMax rightLeader;

    // Simulation specific items 
    DifferentialDrivetrainSim diffDriveSim;
    EncoderSim leftEncoderSim;
    EncoderSim rightEncoderSim;

    // Limiter used to limit how fast velocity is allowed to change
    final SlewRateLimiter limiter = new SlewRateLimiter(1.5);


    public void init() {
        // TODO: renumber these motor controllers with the correct ID's
        leftLeader = new CANSparkMax(1, MotorType.kBrushless);
        motorControllerGroupLeft = new MotorControllerGroup(
        leftLeader,
            new CANSparkMax(3, MotorType.kBrushless));
        
        rightLeader = new CANSparkMax(2, MotorType.kBrushless);
        motorControllerGroupRight = new MotorControllerGroup(
            rightLeader,
            new CANSparkMax(4, MotorType.kBrushless));
=======

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
>>>>>>> 01ea2f83f98265c1c412afd23ff53a949267c9ef

        motorControllerGroupRight.setInverted(true);

        diffDrive = new DifferentialDrive(motorControllerGroupRight, motorControllerGroupLeft);

<<<<<<< HEAD
        gyro = new AHRS(SPI.Port.kMXP);

        leftEncoder = leftLeader.getEncoder();
        rightEncoder = rightLeader.getEncoder();

        // Set the conversion factor for position using computed distance per pulse
        leftEncoder.setPositionConversionFactor(Constants.DriveConstants.kEncoderDistancePerPulse);
        rightEncoder.setPositionConversionFactor(Constants.DriveConstants.kEncoderDistancePerPulse);

        // Set the conversion factor for velocity so that we get meters per second instead of RPMs
        leftEncoder.setVelocityConversionFactor(Constants.DriveConstants.kEncoderDistancePerPulse);
        rightEncoder.setVelocityConversionFactor(Constants.DriveConstants.kEncoderDistancePerPulse);

        // Make sure encoders are reset to 0
        resetEncoders();

        odometry = new DifferentialDriveOdometry(gyro.getRotation2d(), leftEncoder.getPosition(), rightEncoder.getPosition());
        
        if (Robot.isSimulation()) {
            diffDriveSim = new DifferentialDrivetrainSim(
                DriveConstants.kDrivetrainPlant,
                DriveConstants.kDriveGearbox,
                DriveConstants.kDriveGearing,
                DriveConstants.kTrackwidthMeters,
                DriveConstants.kWheelDiameterMeters / 2.0,
                VecBuilder.fill(0, 0, 0.0001, 0.1, 0.1, 0.005, 0.005));

                
            /* 
            leftEncoderSim = new EncoderSim(leftEncoder);
            rightEncoderSim = new EncoderSim(rightEncoder);
            REVPhysicsSim.getInstance().addSparkMax(motorFrontLeft, DCMotor.getNEO(1));
            REVPhysicsSim.getInstance().addSparkMax(motorFrontRight, DCMotor.getNEO(1));
            REVPhysicsSim.getInstance().addSparkMax(motorRearLeft, DCMotor.getNEO(1));
            REVPhysicsSim.getInstance().addSparkMax(motorRearRight, DCMotor.getNEO(1));
            */

            REVPhysicsSim.getInstance().addSparkMax(leftLeader, DCMotor.getNEO(3));
            REVPhysicsSim.getInstance().addSparkMax(rightLeader, DCMotor.getNEO(3));

            
            SmartDashboard.putData("Field", field);

        }
=======
>>>>>>> 01ea2f83f98265c1c412afd23ff53a949267c9ef

    }

    @Override
    public void periodic() {
<<<<<<< HEAD
        odometry.update(gyro.getRotation2d(), leftEncoder.getPosition(), rightEncoder.getPosition());
        field.setRobotPose(odometry.getPoseMeters());

        
        // Log dashboard values
        SmartDashboard.putNumber("Drive Left Position", leftEncoder.getPosition());
        SmartDashboard.putNumber("Drive Right Position", rightEncoder.getPosition());
        SmartDashboard.putNumber("Roll", gyro.getRoll());

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

        REVPhysicsSim.getInstance().run();
        /* 
        leftEncoderSim.setDistance(diffDriveSim.getLeftPositionMeters());
        leftEncoderSim.setRate(diffDriveSim.getLeftVelocityMetersPerSecond());
        rightEncoderSim.setDistance(diffDriveSim.getRightPositionMeters());
        rightEncoderSim.setRate(diffDriveSim.getRightVelocityMetersPerSecond());
        */
        
    }
=======
    

    }

>>>>>>> 01ea2f83f98265c1c412afd23ff53a949267c9ef

    public void drive(double leftOutput, double rightOutput) {
        diffDrive.tankDrive(leftOutput, rightOutput);
    }
<<<<<<< HEAD

    public void resetEncoders()
    {
        leftEncoder.setPosition(0);
        rightEncoder.setPosition(0);
    }

    public Pose2d getPose()
    {
        return odometry.getPoseMeters();
    }

    public DifferentialDriveWheelSpeeds getWheelSpeeds()
    {
        return new DifferentialDriveWheelSpeeds(leftEncoder.getVelocity(), rightEncoder.getVelocity());
    }

    public void resetOdometry(Pose2d pose)
    {
        resetEncoders();
        odometry.resetPosition(gyro.getRotation2d(), leftEncoder.getPosition(), rightEncoder.getPosition(), pose);
    }
    
    public void arcadeDrive(double fwd, double rot)
    {
        diffDrive.arcadeDrive(fwd * dampenFactor, rot);
=======
    
    public void arcadeDrive(double fwd, double rot)
    {
        diffDrive.arcadeDrive(fwd, rot);
>>>>>>> 01ea2f83f98265c1c412afd23ff53a949267c9ef
        //diffDrive.arcadeDrive(limiter.calculate(fwd), rot);
    }

    public void tankDriveVolts(double left, double right)
    {
        motorControllerGroupLeft.setVoltage(left);
        motorControllerGroupRight.setVoltage(right);
        diffDrive.feed();
    }

<<<<<<< HEAD
    public double getAverageEncoderDistance()
    {
        return (leftEncoder.getPosition() + rightEncoder.getPosition()) / 2.0;
    }

=======
>>>>>>> 01ea2f83f98265c1c412afd23ff53a949267c9ef
    public void setMaxOutput(double maxOutput)
    {
        diffDrive.setMaxOutput(maxOutput);
    }

<<<<<<< HEAD
    public void zeroHeading()
    {
        gyro.reset();
    }

    public double getHeading()
    {
        return gyro.getRotation2d().getDegrees();
    }

    public double getRoll()
    {
        return gyro.getRoll();
    }

    public double getTurnRate()
    {
        return -gyro.getRate();
    }

=======
>>>>>>> 01ea2f83f98265c1c412afd23ff53a949267c9ef
    public void setTurnSpeed(double turnSpeed)
    {
        diffDrive.curvatureDrive(0, turnSpeed, true);
    }
<<<<<<< HEAD

    public double getLeftPosition()
    {
        return leftEncoder.getPosition();
    }

    public double getRightPosition()
    {
        return rightEncoder.getPosition();
    }

    public void setDampenFactor(double dampen)
    {
        dampenFactor = dampen;
    }

    public void setBrakeMode(boolean isBrake)
    {
        if (isBrake)
        {
            leftLeader.setIdleMode(IdleMode.kBrake);
        }
        else {
            leftLeader.setIdleMode(IdleMode.kBrake);
        }
    }
=======
>>>>>>> 01ea2f83f98265c1c412afd23ff53a949267c9ef
}
