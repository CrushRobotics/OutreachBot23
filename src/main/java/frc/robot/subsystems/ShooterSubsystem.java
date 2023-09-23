package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.commands.ShooterCommand;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

public class ShooterSubsystem extends SubsystemBase {
    
    private TalonSRX shooterController;
    
    // TODO: this value is placeholder. fix it later
    shooterController = new TalonSRX(1);

    shooterController.setNeutralMode(NeutralMode.Brake);

    public void shoot(boolean button) {
        if(button == true) {
            shooterController.set(ControlMode.PercentOutput, 0.1);
        }
        else {
            shooterController.set(ControlMode.PercentOutput, 0);
        }
    }
}
