    package org.firstinspires.ftc.teamcode.Autos.BlueClose;

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
    import org.firstinspires.ftc.teamcode.MecanumDrive;
    import org.firstinspires.ftc.teamcode.Autos.Coordinates.BlueCloseCoordinated;
    import org.firstinspires.ftc.teamcode.systems.Camera;
    import org.firstinspires.ftc.teamcode.systems.Hood;
    import org.firstinspires.ftc.teamcode.systems.Intake;
    import org.firstinspires.ftc.teamcode.systems.Shooter;
    import org.firstinspires.ftc.teamcode.systems.Transfer;
    import org.firstinspires.ftc.teamcode.systems.Turret;

    import java.util.Arrays;

    @Config
    @Autonomous (name = "BlueClose", group = "autonomus")

    public class BlueCloseWithActions extends LinearOpMode {
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


            MecanumDrive drive = new MecanumDrive(hardwareMap, BlueCloseCoordinated.getStart());
            AutoActions actions = new AutoActions(intake, transfer,  turret, shooter, hood, this);
            Action goToShoot_0 = drive.actionBuilder(BlueCloseCoordinated.getStart())
                    .strafeTo(BlueCloseCoordinated.getShooting().position)
                    .build();
            Action waitToShoot0 = drive.actionBuilder(BlueCloseCoordinated.getShooting())
                    .waitSeconds(2)
                    .build();
            Action waitToShoot1 = drive.actionBuilder(BlueCloseCoordinated.getShooting())
                    .waitSeconds(3)
                    .build();
            Action waitToShoot2 = drive.actionBuilder(BlueCloseCoordinated.getShooting())
                    .waitSeconds(3)
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

            Action goToShoot_2 = drive.actionBuilder(BlueCloseCoordinated.getSecondIntakeEnd())
                    .setTangent(Math.toRadians(70))
                    .splineToLinearHeading(BlueCloseCoordinated.getLastShooting(), BlueCloseCoordinated.getShooting().heading)
                    .build();

            waitForStart();
            hood.setPosition(Hood.UP);
            if (isStopRequested()) return;

            Actions.runBlocking(
                    new ParallelAction(actions.moveTurret(),
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
