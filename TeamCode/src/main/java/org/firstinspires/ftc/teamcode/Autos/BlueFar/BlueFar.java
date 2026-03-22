package org.firstinspires.ftc.teamcode.Autos.BlueFar;

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
import org.firstinspires.ftc.teamcode.Autos.Coordinates.BlueCloseCoordinated;
import org.firstinspires.ftc.teamcode.Autos.Coordinates.BlueFarCoordinates;
import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.Autos.Coordinates.BlueCloseCoordinated;
import org.firstinspires.ftc.teamcode.systems.Camera;
import org.firstinspires.ftc.teamcode.systems.Hood;
import org.firstinspires.ftc.teamcode.systems.Intake;
import org.firstinspires.ftc.teamcode.systems.Shooter;
import org.firstinspires.ftc.teamcode.systems.Transfer;
import org.firstinspires.ftc.teamcode.systems.Turret;
import org.firstinspires.ftc.teamcode.systems.TurretPosition;

import java.util.Arrays;

@Config
@Autonomous (name = "BlueFar  ", group = "autonomus")

public class BlueFar extends LinearOpMode {
    AutoActions actions;
    Intake intake;
    Transfer transfer;
    Hood hood;
    Shooter shooter;
    TurretPosition turret;
    Camera camera;


    @Override
    public void runOpMode() throws InterruptedException {

        intake = new Intake(this);
        transfer = new Transfer(this);
        shooter = new Shooter(this);
        camera = new Camera(this);
        turret = new TurretPosition(this , camera);

        hood =  new Hood(this);
//        MinVelConstraint velCon = new MinVelConstraint(Arrays.asList(new TranslationalVelConstraint(10),new AngularVelConstraint(10)));


        MecanumDrive drive = new MecanumDrive(hardwareMap, BlueFarCoordinates.getStart());
        AutoActions actions = new AutoActions(intake, transfer,  turret, shooter, hood, this);
        Action goToShoot_0 = drive.actionBuilder(BlueFarCoordinates.getStart())
                .splineToLinearHeading(BlueFarCoordinates.getShooting(), BlueFarCoordinates.getShooting().heading)
                .build();
        Action waitToShoot0 = drive.actionBuilder(BlueFarCoordinates.getShooting())
                .waitSeconds(2)
                .build();
        Action waitToShoot1 = drive.actionBuilder(BlueFarCoordinates.getShooting())
                .waitSeconds(3)
                .build();
        Action waitToShoot2 = drive.actionBuilder(BlueFarCoordinates.getShooting())
                .waitSeconds(3)
                .build();


        Action goToCollect_1 = drive.actionBuilder(BlueFarCoordinates.getShooting())
                .setTangent(Math.toRadians(90))
                .splineToLinearHeading(BlueFarCoordinates.getFirstIntakeStart(), BlueFarCoordinates.getFirstIntakeStart().heading)
                .splineToLinearHeading(BlueFarCoordinates.getFirstIntakeEnd(), BlueFarCoordinates.getFirstIntakeEnd().heading)
                .build();

        Action goToShoot_1 = drive.actionBuilder(BlueFarCoordinates.getFirstIntakeEnd())
                .strafeToLinearHeading(BlueFarCoordinates.getShooting().position, BlueFarCoordinates.getShooting().heading)
                .build();

        Action goToCollect_2 = drive.actionBuilder(BlueFarCoordinates.getShooting())
                .splineToLinearHeading(BlueFarCoordinates.getSecondIntakeStart(), BlueFarCoordinates.getSecondIntakeStart().heading)
                .splineToLinearHeading(BlueFarCoordinates.getSecondIntakeEnd(), BlueFarCoordinates.getSecondIntakeEnd().heading)
                .build();

        Action goToShoot_2 = drive.actionBuilder(BlueFarCoordinates.getSecondIntakeEnd())
                .setTangent(Math.toRadians(70))
                .splineToLinearHeading(BlueFarCoordinates.getShooting(), BlueFarCoordinates.getShooting().heading)
                .build();
        Action park = drive.actionBuilder(BlueFarCoordinates.getShooting())
                .strafeToLinearHeading(BlueFarCoordinates.getPark().position, BlueFarCoordinates.getStart().heading)
                .build();

        waitForStart();
        if (isStopRequested()) return;

        Actions.runBlocking(
                new ParallelAction(actions.moveTurretblueFar(),
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
                                )
                        )

                )
        );

    }
}
