package frc.robot.commands;

import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake;

public class RunIntake extends Command {

    public Intake m_intake;
    public RunIntake(Intake intake) {
        m_intake = intake;
        addRequirements(intake);
    }
    @Override
    public void execute() {
        m_intake.forward();
  }
}
