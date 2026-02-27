package org.firstinspires.ftc.teamcode.Autos.RedClose;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.AngularVelConstraint;
import com.acmerobotics.roadrunner.MinVelConstraint;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TranslationalVelConstraint;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Autos.AutoActions;
import org.firstinspires.ftc.teamcode.Autos.Coordinates.RedCloseCoordinates;
import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.Autos.Coordinates.RedCloseCoordinates;
import org.firstinspires.ftc.teamcode.systems.Camera;
import org.firstinspires.ftc.teamcode.systems.Hood;
import org.firstinspires.ftc.teamcode.systems.Intake;
import org.firstinspires.ftc.teamcode.systems.Shooter;
import org.firstinspires.ftc.teamcode.systems.Transfer;
import org.firstinspires.ftc.teamcode.systems.Turret;

import java.util.Arrays;

@Config
@Autonomous (name = "RedClose", group = "autonomus")

public class RedCloseWithActions extends LinearOpMode {
    AutoActions actions;
    Intake intake;
    Transfer transfer;
    Hood hood;
    Shooter shooter;
    Turret turret;
    Camera camera;


    @Override
    public void runOpMode() throws InterruptedException {

        intake = new Intake(this);
        transfer = new Transfer(this);
        shooter = new Shooter(this);
        camera = new Camera(this);
        turret = new Turret(this, null, camera);
        turret.init();
        hood =  new Hood(this);
        MinVelConstraint velCon = new MinVelConstraint(Arrays.asList(new TranslationalVelConstraint(10),new AngularVelConstraint(10)));


        MecanumDrive drive = new MecanumDrive(hardwareMap, RedCloseCoordinates.getStart());
        AutoActions actions = new AutoActions(intake, transfer,  turret, shooter, hood, this);
        Action goToShoot_0 = drive.actionBuilder(RedCloseCoordinates.getStart())
                .strafeTo(RedCloseCoordinates.getShooting().position)
                .build();
        Action waitToShoot0 = drive.actionBuilder(RedCloseCoordinates.getShooting())
                .waitSeconds(2)
                .build();
        Action waitToShoot1 = drive.actionBuilder(RedCloseCoordinates.getShooting())
                .waitSeconds(3)
                .build();
        Action waitToShoot2 = drive.actionBuilder(RedCloseCoordinates.getShooting())
                .waitSeconds(3)
                .build();


        Action goToCollect_1 = drive.actionBuilder(RedCloseCoordinates.getShooting())
                .setTangent(Math.toRadians(90))
                .splineToLinearHeading(RedCloseCoordinates.getFirstIntakeStart(), RedCloseCoordinates.getFirstIntakeStart().heading)
                .splineToLinearHeading(RedCloseCoordinates.getFirstIntakeEnd(), RedCloseCoordinates.getFirstIntakeEnd().heading)
                .build();

        Action goToShoot_1 = drive.actionBuilder(RedCloseCoordinates.getFirstIntakeEnd())
                .strafeToLinearHeading(RedCloseCoordinates.getShooting().position, RedCloseCoordinates.getShooting().heading)
                .build();

        Action goToCollect_2 = drive.actionBuilder(RedCloseCoordinates.getShooting())
                .splineToLinearHeading(RedCloseCoordinates.getSecondIntakeStart(), RedCloseCoordinates.getSecondIntakeStart().heading)
                .splineToLinearHeading(RedCloseCoordinates.getSecondIntakeEnd(), RedCloseCoordinates.getSecondIntakeEnd().heading)
                .build();

        Action goToShoot_2 = drive.actionBuilder(RedCloseCoordinates.getSecondIntakeEnd())
                .setTangent(Math.toRadians(70))
                .splineToLinearHeading(RedCloseCoordinates.getShooting(), RedCloseCoordinates.getShooting().heading)
                .build();

        waitForStart();
        hood.setPosition(Hood.UP);
        if (isStopRequested()) return;

        Actions.runBlocking(
                new ParallelAction(actions.moveTurretRedClose(),
                        new SequentialAction(
                                new ParallelAction(
                                        goToShoot_0,
                                        actions.shooterStart(),
                                        new SequentialAction(
                                                waitToShoot0,
                                                actions.transferStart()
                                        )
                                ),

                                new ParallelAction(
                                        actions.shooterEnd(),
                                        actions.transferEnd(),
                                        actions.intakeStart(),
                                        goToCollect_1
                                ),
                                new ParallelAction(
                                        goToShoot_1,
                                        actions.intakeEnd()),
                                new ParallelAction(
                                        actions.shooterStartSecond(),
                                        actions.transferStart()
                                ),
                                waitToShoot1,
                                actions.shooterEnd(),
                                actions.transferEnd(),
                                actions.intakeStart(),
                                goToCollect_2,
                                new ParallelAction(
                                        actions.intakeEnd(),
                                        goToShoot_2),
                                new ParallelAction(
                                        actions.shooterStart(),
                                        actions.transferStart()),
                                waitToShoot0,
                                actions.shooterEnd(),
                                actions.transferEnd()

                        )
                )
        );

    }
}
