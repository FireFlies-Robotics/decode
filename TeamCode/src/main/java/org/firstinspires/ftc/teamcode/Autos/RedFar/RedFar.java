package org.firstinspires.ftc.teamcode.Autos.RedFar;

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
import org.firstinspires.ftc.teamcode.Autos.Coordinates.BlueFarCoordinates;
import org.firstinspires.ftc.teamcode.Autos.Coordinates.RedFarCoordinates;
import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.systems.Camera;
import org.firstinspires.ftc.teamcode.systems.Hood;
import org.firstinspires.ftc.teamcode.systems.Intake;
import org.firstinspires.ftc.teamcode.systems.Shooter;
import org.firstinspires.ftc.teamcode.systems.Transfer;
import org.firstinspires.ftc.teamcode.systems.Turret;

import java.util.Arrays;

@Config
@Autonomous (name = "RedFar_3_plus_3", group = "autonomus")

public class RedFar extends LinearOpMode {
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


        MecanumDrive drive = new MecanumDrive(hardwareMap, RedFarCoordinates.getStart());
        AutoActions actions = new AutoActions(intake, transfer,  turret, shooter, hood, this);
        Action goToShoot_0 = drive.actionBuilder(RedFarCoordinates.getStart())
                .splineToLinearHeading(RedFarCoordinates.getShooting(), RedFarCoordinates.getShooting().heading)
                .build();
        Action waitToShoot0 = drive.actionBuilder(RedFarCoordinates.getShooting())
                .waitSeconds(2)
                .build();
        Action waitToShoot1 = drive.actionBuilder(RedFarCoordinates.getShooting())
                .waitSeconds(3)
                .build();
        Action waitToShoot2 = drive.actionBuilder(RedFarCoordinates.getShooting())
                .waitSeconds(3)
                .build();


        Action goToCollect_1 = drive.actionBuilder(RedFarCoordinates.getShooting())
                .setTangent(Math.toRadians(90))
                .splineToLinearHeading(RedFarCoordinates.getFirstIntakeStart(), RedFarCoordinates.getFirstIntakeStart().heading)
                .splineToLinearHeading(RedFarCoordinates.getFirstIntakeEnd(), RedFarCoordinates.getFirstIntakeEnd().heading)
                .build();

        Action goToShoot_1 = drive.actionBuilder(RedFarCoordinates.getFirstIntakeEnd())
                .strafeToLinearHeading(RedFarCoordinates.getShooting().position, RedFarCoordinates.getShooting().heading)
                .build();

        Action goToCollect_2 = drive.actionBuilder(RedFarCoordinates.getShooting())
                .splineToLinearHeading(RedFarCoordinates.getSecondIntakeStart(), RedFarCoordinates.getSecondIntakeStart().heading)
                .splineToLinearHeading(RedFarCoordinates.getSecondIntakeEnd(), RedFarCoordinates.getSecondIntakeEnd().heading)
                .build();

        Action goToShoot_2 = drive.actionBuilder(RedFarCoordinates.getSecondIntakeEnd())
                .setTangent(Math.toRadians(-70))
                .splineToLinearHeading(RedFarCoordinates.getShooting(), RedFarCoordinates.getShooting().heading)
                .build();
        Action park = drive.actionBuilder(RedFarCoordinates.getShooting())
                .strafeToLinearHeading(RedFarCoordinates.getPark().position, RedFarCoordinates.getStart().heading)
                .build();

        waitForStart();
        if (isStopRequested()) return;

        Actions.runBlocking(
                new ParallelAction(actions.moveTurretRedFar(),
                        new SequentialAction(
                                goToShoot_0,
                                new ParallelAction(
                                        actions.shooterStartFar(),
                                        actions.transferStartFar()
                                ),
                                actions.shooterEnd(),
                                actions.transferEnd(),
                                actions.intakeStart(),
                                goToCollect_1,
                                goToShoot_1,
                                new ParallelAction(
                                        actions.shooterStartFar(),
                                        actions.transferStartFar()
                                ),
                                park
                        )

                )
        );

    }
}
