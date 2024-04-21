package frc.robot.commands;

import static frc.robot.Constants.IndexConstants.INDEX_RUN_PCT;
import static frc.robot.Constants.IntakeConstants.INTAKE_RUN_PCT;
import static frc.robot.Constants.IntakeConstants.IS_INTAKING;
import static frc.robot.Constants.PivotConstants.IS_SHUTTLING;
import static frc.robot.Constants.PivotConstants.PIVOT_STOW_POS;
import static frc.robot.Constants.ShooterConstants.IS_AMP;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.commands.index.RunIndex;
import frc.robot.commands.intake.RunIntake;
import frc.robot.commands.pivot.SetPivotPos;
import frc.robot.subsystems.Index;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Pivot;

public class IntakeNote extends ParallelCommandGroup {
  public IntakeNote(Intake intake, Index index, Pivot pivot) {
    addCommands(
        Commands.runOnce(
                () -> {
                  IS_AMP = false;
                  IS_SHUTTLING = false;
                  IS_INTAKING = true;
                })
            .alongWith(
                new SetPivotPos(pivot, PIVOT_STOW_POS)
                    .alongWith(
                        new RunIntake(intake, INTAKE_RUN_PCT)
                            .alongWith(new RunIndex(index, INDEX_RUN_PCT))
                            .until(() -> index.beamBroken() || !IS_INTAKING))));
  }
}
