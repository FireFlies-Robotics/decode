package org.firstinspires.ftc.teamcode.Autos.BlueClose;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.AngularVelConstraint;
import com.acmerobotics.roadrunner.MinVelConstraint;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TranslationalVelConstraint;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Autos.Coordinates.BlueCloseCoordinated;
import org.firstinspires.ftc.teamcode.MecanumDrive;

import java.util.Arrays;

@Config
@Disabled
@Autonomous (name = "BlueClose_3_plus_6", group = "autonomus")

public class BlueClose_3_plus_6 extends LinearOpMode {


    @Override
    public void runOpMode() throws InterruptedException {
        MinVelConstraint velCon = new MinVelConstraint(Arrays.asList(new TranslationalVelConstraint(10),new AngularVelConstraint(10)));


        MecanumDrive drive = new MecanumDrive(hardwareMap, BlueCloseCoordinated.getStart());
        Action goToShoot_0 = drive.actionBuilder(BlueCloseCoordinated.getStart())
                .strafeTo(BlueCloseCoordinated.getShooting().position)
                .build();

        Action goToCollect_1 = drive.actionBuilder(BlueCloseCoordinated.getShooting())
                .setTangent(Math.toRadians(90))
                .splineToLinearHeading(BlueCloseCoordinated.getFirstIntakeStart(), BlueCloseCoordinated.getFirstIntakeStart().heading)
                .splineToLinearHeading(BlueCloseCoordinated.getFirstIntakeEnd(), BlueCloseCoordinated.getFirstIntakeEnd().heading)
                .build();

        Action goToShoot_1 = drive.actionBuilder(BlueCloseCoordinated.getFirstIntakeEnd())
                .strafeToLinearHeading(BlueCloseCoordinated.getShooting().position, BlueCloseCoordinated.getShooting().heading)
                .build();

        Action goToCollect_2 = drive.actionBuilder(BlueCloseCoordinated.getShooting())
                .splineToLinearHeading(BlueCloseCoordinated.getSecondIntakeStart(), BlueCloseCoordinated.getSecondIntakeStart().heading)
                .splineToLinearHeading(BlueCloseCoordinated.getSecondIntakeEnd(), BlueCloseCoordinated.getSecondIntakeEnd().heading)
                .build();

        Action goToShoot_2 = drive.actionBuilder(BlueCloseCoordinated.getStart())
                .strafeToLinearHeading(BlueCloseCoordinated.getShooting().position, BlueCloseCoordinated.getShooting().heading)
                .build();

        waitForStart();

        if (isStopRequested()) return;

        Actions.runBlocking(
                new SequentialAction(
                        goToShoot_0,
                        goToCollect_1,
                        goToShoot_1,
                        goToCollect_2,
                        goToShoot_2
                )

        );

    }
}
