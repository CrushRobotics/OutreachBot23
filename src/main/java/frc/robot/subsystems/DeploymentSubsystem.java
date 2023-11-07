package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

public class DeploymentSubsystem extends SubsystemBase {
    
    TalonSRX motor;

    public DeploymentSubsystem() {
        motor = new TalonSRX(5);

    }

    public void deploy() {
        motor.set(ControlMode.PercentOutput, 0.6);
    }
    
    public void stop() {
        motor.set(ControlMode.PercentOutput, 0);
    }
}
