package frc.robot.subsystems;
import com.revrobotics.CANSparkFlex;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase {
    public CANSparkFlex topMotor;

    public Intake() {
        topMotor = new CANSparkFlex(50, CANSparkFlex.MotorType.kBrushless);
    }

    public void backward(){
        topMotor.set(-.25);
    }
    
    public void forward(){
        topMotor.set(0.25);
    }
}