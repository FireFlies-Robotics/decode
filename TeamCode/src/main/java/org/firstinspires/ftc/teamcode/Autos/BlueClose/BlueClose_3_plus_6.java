package org.firstinspires.ftc.teamcode.Autos.BlueClose;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.AngularVelConstraint;
import com.acmerobotics.roadrunner.MinVelConstraint;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TranslationalVelConstraint;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.Autos.Coordinates.RedCloseCoodrinates;

import java.util.Arrays;

@Config
@Autonomous (name = "BlueClose_3_plus_6", group = "autonomus")

public class BlueClose_3_plus_6 extends LinearOpMode {


    @Override
    public void runOpMode() throws InterruptedException {
        MinVelConstraint velCon = new MinVelConstraint(Arrays.asList(new TranslationalVelConstraint(10),new AngularVelConstraint(10)));


        MecanumDrive drive = new MecanumDrive(hardwareMap, RedCloseCoodrinates.getStart());
        Action goToShoot_0 = drive.actionBuilder(RedCloseCoodrinates.getStart())
                .strafeTo(RedCloseCoodrinates.getShooting().position)
                .build();

        Action goToCollect_1 = drive.actionBuilder(RedCloseCoodrinates.getShooting())
                .setTangent(Math.toRadians(90))
                .splineToLinearHeading(RedCloseCoodrinates.getFirstIntakeStart(), RedCloseCoodrinates.getFirstIntakeStart().heading)
                .splineToLinearHeading(RedCloseCoodrinates.getFirstIntakeEnd(), RedCloseCoodrinates.getFirstIntakeEnd().heading)
                .build();

        Action goToShoot_1 = drive.actionBuilder(RedCloseCoodrinates.getFirstIntakeEnd())
                .strafeToLinearHeading(RedCloseCoodrinates.getShooting().position, RedCloseCoodrinates.getShooting().heading)
                .build();

        Action goToCollect_2 = drive.actionBuilder(RedCloseCoodrinates.getShooting())
                .splineToLinearHeading(RedCloseCoodrinates.getSecondIntakeStart(), RedCloseCoodrinates.getSecondIntakeStart().heading)
                .splineToLinearHeading(RedCloseCoodrinates.getSecondIntakeEnd(), RedCloseCoodrinates.getSecondIntakeEnd().heading)
                .build();

        Action goToShoot_2 = drive.actionBuilder(RedCloseCoodrinates.getStart())
                .strafeToLinearHeading(RedCloseCoodrinates.getShooting().position, RedCloseCoodrinates.getShooting().heading)
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
