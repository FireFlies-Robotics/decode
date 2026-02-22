package org.firstinspires.ftc.teamcode.Autos.BlueClose;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.AngularVelConstraint;
import com.acmerobotics.roadrunner.MinVelConstraint;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TranslationalVelConstraint;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.arcrobotics.ftclib.command.WaitUntilCommand;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.onbotjava.handlers.objbuild.WaitForBuild;
import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.Autos.Coordinates.BlueCloseCoordianates;

import java.util.Arrays;

@Config
@Autonomous (name = "BlueClose_3_plus_6", group = "autonomus")

public class BlueClose_3_plus_6 extends LinearOpMode {


    @Override
    public void runOpMode() throws InterruptedException {
        MinVelConstraint velCon = new MinVelConstraint(Arrays.asList(new TranslationalVelConstraint(10),new AngularVelConstraint(10)));


        MecanumDrive drive = new MecanumDrive(hardwareMap, BlueCloseCoordianates.getStart());
        Action goToShoot_0 = drive.actionBuilder(BlueCloseCoordianates.getStart())
                .strafeTo(BlueCloseCoordianates.getShooting().position)
                .build();

        Action goToCollect_1 = drive.actionBuilder(BlueCloseCoordianates.getShooting())
                .setTangent(Math.toRadians(90))
                .splineToLinearHeading(BlueCloseCoordianates.getFirstIntakeStart(), BlueCloseCoordianates.getFirstIntakeStart().heading)
                .splineToLinearHeading(BlueCloseCoordianates.getFirstIntakeEnd(), BlueCloseCoordianates.getFirstIntakeEnd().heading)
                .build();

        Action goToShoot_1 = drive.actionBuilder(BlueCloseCoordianates.getFirstIntakeEnd())
                .strafeToLinearHeading(BlueCloseCoordianates.getShooting().position, BlueCloseCoordianates.getShooting().heading)
                .build();

        Action goToCollect_2 = drive.actionBuilder(BlueCloseCoordianates.getShooting())
                .splineToLinearHeading(BlueCloseCoordianates.getSecondIntakeStart(), BlueCloseCoordianates.getSecondIntakeStart().heading)
                .splineToLinearHeading(BlueCloseCoordianates.getSecondIntakeEnd(), BlueCloseCoordianates.getSecondIntakeEnd().heading)
                .build();

        Action goToShoot_2 = drive.actionBuilder(BlueCloseCoordianates.getStart())
                .strafeToLinearHeading(BlueCloseCoordianates.getShooting().position, BlueCloseCoordianates.getShooting().heading)
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
