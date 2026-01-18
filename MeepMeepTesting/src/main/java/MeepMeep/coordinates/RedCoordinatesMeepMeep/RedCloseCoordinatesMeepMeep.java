package MeepMeep.coordinates.RedCoordinatesMeepMeep;

import com.acmerobotics.roadrunner.Pose2d;

public class RedCloseCoordinatesMeepMeep {

    private static final double startX = -50;
    private static final double startY = 50;
    private static final double startPoseHeading =
            Math.toRadians(218) + Math.PI + Math.PI / 2;

    private static final double firstIntakeStartX = -12.5;
    private static final double intakeStartY = 35;
    private static final double secondIntakeX = 11.5;

    private static final double firstIntakeStartHeading =
            Math.toRadians(270) + Math.PI;

    private static final double intakeSEndY = 50;

    private static final double shootingX = -20;
    private static final double shootingY = 15;

    // FIX: -90 degrees to compensate for strafeToSplineHeading
    private static final double shootingHeading =
            Math.toRadians(225) + Math.PI + Math.PI / 2;

    private static final Pose2d start =
            new Pose2d(startX, startY, startPoseHeading);

    private static final Pose2d firstIntakeStart =
            new Pose2d(firstIntakeStartX, intakeStartY, firstIntakeStartHeading);

    private static final Pose2d firstIntakeEnd =
            new Pose2d(firstIntakeStartX, intakeSEndY, firstIntakeStartHeading);

    private static final Pose2d secondIntakeStart =
            new Pose2d(secondIntakeX, intakeStartY, firstIntakeStartHeading);

    private static final Pose2d secondIntakeEnd =
            new Pose2d(secondIntakeX, intakeSEndY, firstIntakeStartHeading);

    private static final Pose2d shooting =
            new Pose2d(shootingX, shootingY, shootingHeading);

    public static Pose2d getStart() {
        return start;
    }

    public static Pose2d getFirstIntakeStart() {
        return firstIntakeStart;
    }

    public static Pose2d getFirstIntakeEnd() {
        return firstIntakeEnd;
    }

    public static Pose2d getSecondIntakeStart() {
        return secondIntakeStart;
    }

    public static Pose2d getSecondIntakeEnd() {
        return secondIntakeEnd;
    }

    public static Pose2d getShooting() {
        return shooting;
    }
}
