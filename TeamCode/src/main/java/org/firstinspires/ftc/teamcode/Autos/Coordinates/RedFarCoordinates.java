package org.firstinspires.ftc.teamcode.Autos.Coordinates;

import com.acmerobotics.roadrunner.Pose2d;

public class RedFarCoordinates {

    private static final double startX = 60.5;

    private static final double startY = 13.6;
    private static final double startPoseHeading = Math.toRadians(180);

    private static final double firstIntakeStartX = 34.5;
    private static final double intakeStartY = 30;
    private static final double secondIntakeX = 11.5;

    private static final double firstIntakeStartHeading = Math.toRadians(90);
    private static final double intakeSEndY = 54;


    private static final double shootingX = 57;
    private static final double shootingY = 14;
    private static final double shootingHeading = Math.toRadians(160);



    private static final Pose2d start = new Pose2d(startX, startY, startPoseHeading);
    private static final Pose2d firstIntakeStart = new Pose2d(firstIntakeStartX, intakeStartY, firstIntakeStartHeading);
    private static final Pose2d secondIntakeStart = new Pose2d(secondIntakeX, intakeStartY, firstIntakeStartHeading);
    private static final Pose2d secondIntakeEnd = new Pose2d(secondIntakeX, intakeSEndY, firstIntakeStartHeading);
    private static final Pose2d park = new Pose2d(shootingX-7, shootingY, -90);

    private static final Pose2d firstIntakeEnd = new Pose2d(firstIntakeStartX, intakeSEndY, firstIntakeStartHeading);



    private static final Pose2d shooting = new Pose2d(shootingX, shootingY, shootingHeading);


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
    public static Pose2d getPark (){return  park;}

    public static Pose2d getShooting(){
        return shooting;
    }
}
