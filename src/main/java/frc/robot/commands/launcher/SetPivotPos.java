package frc.robot.commands.launcher;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ShooterPivot;

public class SetPivotPos extends Command {

    ShooterPivot m_pivot;
    double position;

    public SetPivotPos(ShooterPivot pivot, double pos) {

        m_pivot = pivot;
        position = pos;
        addRequirements(m_pivot);


    }
    @Override
    public void execute() {
        m_pivot.setPosition(position);
    }
    
}
