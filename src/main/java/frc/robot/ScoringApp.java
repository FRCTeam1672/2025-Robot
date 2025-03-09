package frc.robot;

import edu.wpi.first.networktables.IntegerSubscriber;
import edu.wpi.first.networktables.IntegerTopic;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.PubSubOption;
import edu.wpi.first.networktables.StringSubscriber;
import edu.wpi.first.networktables.StringTopic;

public class ScoringApp {
    private static ScoringApp instance;

    private IntegerSubscriber coralLevel;
    private IntegerSubscriber algaeLevel;
    private StringSubscriber reefSide;
    private IntegerSubscriber coralStation;

    public ScoringApp() {
        IntegerTopic coralIntegerTopic = NetworkTableInstance.getDefault().getIntegerTopic("/AppScoring/CoralLevel");
        IntegerTopic algaeIntegerTopic = NetworkTableInstance.getDefault().getIntegerTopic("/AppScoring/AlgaeLevel");
        StringTopic reefStringTopic = NetworkTableInstance.getDefault().getStringTopic("/AppScoring/ReefSide");
        IntegerTopic coralIntegerTopic2 = NetworkTableInstance.getDefault().getIntegerTopic("/AppScoring/CoralStation");
        coralLevel = coralIntegerTopic.subscribe(-1, PubSubOption.sendAll(true));
        algaeLevel = algaeIntegerTopic.subscribe(-1, PubSubOption.sendAll(true));
        reefSide = reefStringTopic.subscribe("A", PubSubOption.sendAll(true));
        coralStation = coralIntegerTopic2.subscribe(-1, PubSubOption.sendAll(true));
    }

    public static ScoringApp getInstance() {
        if (instance == null)
            instance = new ScoringApp();
        return instance;
    }

    public int getCoralLevel() {
        return (int)coralLevel.get();
    }

    public int getAlgaeLevel() {
        return (int)algaeLevel.get();
    }

    public String getReefSide() {
        return reefSide.get();
    }

    public int getCoralStation() {
        return (int)coralStation.get();
    }

    public int getAlgaeSide() {
        if (getReefSide().equals("A")||getReefSide().equals("B")) return 1;
        if (getReefSide().equals("C")||getReefSide().equals("D")) return 2;
        if (getReefSide().equals("E")||getReefSide().equals("F")) return 3;
        if (getReefSide().equals("G")||getReefSide().equals("H")) return 4;
        if (getReefSide().equals("I")||getReefSide().equals("J")) return 5;
        if (getReefSide().equals("K")||getReefSide().equals("L")) return 6;
        if (algaeLevel.get() == 0) return 0;
        return -1;
    }
}
