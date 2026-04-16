package frc.robot.commands;

import java.util.function.BooleanSupplier;

import edu.wpi.first.networktables.BooleanEntry;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StringEntry;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;

public class MatchStateCommand extends Command {

    private static final double TOTAL_TIME = 140.0;

    private static final double TRANSITION_TIME = 10.0;
    private static final double SHIFT_TIME = 25.0;
    private static final double ENDGAME_TIME = 30.0;

    private double matchStartTime = 0.0;

    // 0=transition, 1=shift1, 2=shift2, 3=shift3, 4=shift4, 5=endgame
    private int currentPhase = 0;
    private double phaseStartTime = 0.0;

    private final BooleanSupplier isAutoWinSupplier;

    private final StringEntry matchTimeEntry;
    private final StringEntry shiftTimeEntry;
    private final StringEntry matchPhaseEntry;
    private final BooleanEntry hubActiveEntry;

    public MatchStateCommand(BooleanSupplier isAutoWinSupplier) {
        this.isAutoWinSupplier = isAutoWinSupplier;

        NetworkTable table = NetworkTableInstance.getDefault().getTable("Elastic");

        matchTimeEntry = table.getStringTopic("MatchTime").getEntry("02:20");
        shiftTimeEntry = table.getStringTopic("ShiftTimeText").getEntry("00:00");
        matchPhaseEntry = table.getStringTopic("MatchPhase").getEntry("transition");
        hubActiveEntry = table.getBooleanTopic("isHubActive").getEntry(false);
    }

    @Override
    public void initialize() {
        matchStartTime = Timer.getFPGATimestamp();
        currentPhase = 0;
        phaseStartTime = matchStartTime;
        updateAll();
    }

    @Override
    public void execute() {
        double now = Timer.getFPGATimestamp();

        double elapsedPhase = now - phaseStartTime;
        double duration = getPhaseDuration(currentPhase);

        if (elapsedPhase >= duration) {
            currentPhase++;
            if (currentPhase <= 5) {
                phaseStartTime = now;
            }
        }

        updateAll();
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        matchTimeEntry.set("00:00");
        shiftTimeEntry.set("00:00");
        matchPhaseEntry.set("end");
        hubActiveEntry.set(false);
    }

    private void updateAll() {
        double now = Timer.getFPGATimestamp();

        double matchElapsed = now - matchStartTime;
        double matchRemaining = Math.max(0.0, TOTAL_TIME - matchElapsed);
        matchTimeEntry.set(formatTime(matchRemaining));

        double shiftRemaining = getMergedRemainingTime(now);
        shiftTimeEntry.set(formatTime(shiftRemaining));

        matchPhaseEntry.set(getPhaseName(currentPhase));

        hubActiveEntry.set(isPhaseActive(currentPhase));
    }

    private double getMergedRemainingTime(double now) {
        double elapsed = now - phaseStartTime;
        double currentRemaining = Math.max(0.0, getPhaseDuration(currentPhase) - elapsed);

        double total = currentRemaining;

        if (isPhaseActive(currentPhase)) {
            int next = currentPhase + 1;
            while (next <= 5 && isPhaseActive(next)) {
                total += getPhaseDuration(next);
                next++;
            }
        }

        return total;
    }

    private boolean isPhaseActive(int phase) {
        boolean isAutoWin = isAutoWinSupplier.getAsBoolean();

        switch (phase) {
            case 0:
            case 5:
                return true;
            case 1:
                return !isAutoWin;
            case 2:
                return isAutoWin;
            case 3:
                return !isAutoWin;
            case 4:
                return isAutoWin;
            default:
                return false;
        }
    }

    private double getPhaseDuration(int phase) {
        switch (phase) {
            case 0:
                return TRANSITION_TIME;
            case 1:
            case 2:
            case 3:
            case 4:
                return SHIFT_TIME;
            case 5:
                return ENDGAME_TIME;
            default:
                return 0.0;
        }
    }

    private String getPhaseName(int phase) {
        switch (phase) {
            case 0:
                return "TRANSITION";
            case 1:
                return "SHIFT1";
            case 2:
                return "SHIFT2";
            case 3:
                return "SHIFT3";
            case 4:
                return "SHIFT4";
            case 5:
                return "ENDGAME";
            default:
                return "END";
        }
    }

    private String formatTime(double time) {
        int totalSeconds = (int) Math.ceil(time);
        if (totalSeconds < 0) totalSeconds = 0;
        int minutes = totalSeconds / 60;
        int seconds = totalSeconds % 60;
        return String.format("%02d:%02d", minutes, seconds);
    }
}
