package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ClimberSubsystem extends SubsystemBase{
 
    TalonFX climber = new TalonFX(40);
    public ClimberSubsystem(){

    climber.setNeutralMode(NeutralModeValue.Brake);
    }

    public boolean toggletest = true;

    public void ClimbCommand() {
    if (toggletest = false) {
        toggletest = true;
        climber.set(Constants.Climber.ClimberDown);
        new WaitCommand(Constants.Climber.ClimberRunDuration);
        stopMotors();
    } else {

        toggletest = false;
        climber.set(Constants.Climber.ClimberUp);
        new WaitCommand(Constants.Climber.ClimberRunDuration);
        stopMotors();

    }
    }

  

    public void stopMotors(){

        climber.stopMotor();

    }

}
