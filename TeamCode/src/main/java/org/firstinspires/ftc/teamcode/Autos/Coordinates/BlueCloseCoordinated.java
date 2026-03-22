package org.firstinspires.ftc.teamcode.Autos.Coordinates;

import com.acmerobotics.roadrunner.Pose2d;

public class BlueCloseCoordinated {

    private static final double startX = -60.6634;
    private static final double startY = -43.3425;
    private static final double startPoseHeading = Math.toRadians(-136);

    private static final double firstIntakeStartX = -16.8148;
    private static final double intakeStartY = -26.1426;
    private static final double intakeSEndY = -53.5;

    private static final double secondIntakeX = 9.8;

    private static final double firstIntakeStartHeading = Math.toRadians(-91);


    private static final double shootingX = -18;
    private static final double shootingY = -11;
    private static final double shootingHeading = Math.toRadians(225);



    private static final Pose2d start = new Pose2d(startX, startY, startPoseHeading);
    private static final Pose2d firstIntakeStart = new Pose2d(firstIntakeStartX, intakeStartY, firstIntakeStartHeading);

    private static final Pose2d secondIntakeStart = new Pose2d(secondIntakeX, intakeStartY, firstIntakeStartHeading);
    private static final Pose2d secondIntakeEnd = new Pose2d(secondIntakeX+2.4, intakeSEndY+3, firstIntakeStartHeading);


    private static final Pose2d firstIntakeEnd = new Pose2d(firstIntakeStartX, intakeSEndY, firstIntakeStartHeading);



    private static final Pose2d shooting = new Pose2d(shootingX, shootingY, shootingHeading);
    private static final Pose2d lastShooting = new Pose2d(shootingX-7.5, shootingY, shootingHeading);



    public static Pose2d getStart() {
        return start;
    }

    public static Pose2d getFirstIntakeStart() {return firstIntakeStart;
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

    public static Pose2d getShooting(){
        return shooting;
    }
    public static Pose2d getLastShooting(){
        return lastShooting;
    }

}
