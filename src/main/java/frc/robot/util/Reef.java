package frc.robot.util;

import frc.robot.util.ReefAlignment;
public class Reef {
    public static ReefAlignment fromSide(String side) {
        side = side.toLowerCase();
        return switch (side) {
            case "a" -> new ReefAlignment(ReefOrientation.FRONT, ReefSide.LEFT);
            case "b" -> new ReefAlignment(ReefOrientation.FRONT, ReefSide.RIGHT);
            case "c" -> new ReefAlignment(ReefOrientation.FRONT_RIGHT, ReefSide.LEFT);
            case "d" -> new ReefAlignment(ReefOrientation.FRONT_RIGHT, ReefSide.RIGHT);
            case "e" -> new ReefAlignment(ReefOrientation.BACK_RIGHT, ReefSide.LEFT);
            case "f" -> new ReefAlignment(ReefOrientation.BACK_RIGHT, ReefSide.RIGHT);
            case "g" -> new ReefAlignment(ReefOrientation.BACK, ReefSide.LEFT);
            case "h" -> new ReefAlignment(ReefOrientation.BACK, ReefSide.RIGHT);
            case "i" -> new ReefAlignment(ReefOrientation.BACK_LEFT, ReefSide.LEFT);
            case "j" -> new ReefAlignment(ReefOrientation.BACK_LEFT, ReefSide.RIGHT);
            case "k" -> new ReefAlignment(ReefOrientation.FRONT_LEFT, ReefSide.LEFT);
            case "l" -> new ReefAlignment(ReefOrientation.FRONT_LEFT, ReefSide.RIGHT);
            default -> throw new IllegalArgumentException("Illegal side.");
        };

    }
}

enum ReefSide {
    LEFT,
    RIGHT
}

enum ReefOrientation {

    FRONT(0, 0),
    FRONT_RIGHT(60, 60),
    BACK_RIGHT (120, 120),
    BACK (180, 180),
    FRONT_LEFT (-120, 240),
    BACK_LEFT (-60, 300),
    ;


    private final int robotRotation;
    private final int reefRotation;
    ReefOrientation(int robotRotation, int reefRotation) {
        this.robotRotation = robotRotation;
        this.reefRotation = reefRotation;
    }

    public int getRobotRotation() {
        return robotRotation;
    }


    public int getReefRotation() {
        return reefRotation;
    }
}
