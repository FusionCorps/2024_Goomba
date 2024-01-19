package frc.robot.subsystems;
import com.revrobotics.CANSparkFlex;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase {
    
    public CANSparkFlex TopMotor;

    public Intake() {

    }

    public void backward(){
      TopMotor.set(-.25);
    }
    
    public void forward(){
      TopMotor.set(0.25);
    }
}