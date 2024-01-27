package frc.robot.subsystems;
import com.revrobotics.CANSparkFlex;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Intake extends SubsystemBase {
    private CANSparkFlex topMotor;

    public Intake() {
        topMotor = new CANSparkFlex(Constants.INTAKE_MOTOR_ID, CANSparkFlex.MotorType.kBrushless);
    }

    public void backward(){
        topMotor.set(-0.25);
    }
    
    public void forward(){
        topMotor.set(0.25);
    }
}