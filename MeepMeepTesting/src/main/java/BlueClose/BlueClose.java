package BlueClose;

import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

import MeepMeep.coordinates.BlueCoordinates.BlueCloseCoordinatesMeepMeep;

public class BlueClose {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(670);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .build();

        myBot.runAction(myBot.getDrive().actionBuilder(BlueCloseCoordinatesMeepMeep.getStart())
                .setTangent(Math.toRadians(45))
                .splineToLinearHeading(BlueCloseCoordinatesMeepMeep.getFirstIntakeStart(), BlueCloseCoordinatesMeepMeep.getFirstIntakeStart().heading)
                .splineToSplineHeading(BlueCloseCoordinatesMeepMeep.getFirstIntakeEnd(), BlueCloseCoordinatesMeepMeep.getFirstIntakeEnd().heading)

                //first intake

                .strafeToLinearHeading(BlueCloseCoordinatesMeepMeep.getShooting().position, BlueCloseCoordinatesMeepMeep.getShooting().heading)
                .waitSeconds(2)
                //first shoot

                .splineToLinearHeading(BlueCloseCoordinatesMeepMeep.getSecondIntakeStart(), BlueCloseCoordinatesMeepMeep.getSecondIntakeStart().heading)
                .strafeToSplineHeading(BlueCloseCoordinatesMeepMeep.getSecondIntakeEnd().position, BlueCloseCoordinatesMeepMeep.getSecondIntakeEnd().heading)
                .strafeToLinearHeading(BlueCloseCoordinatesMeepMeep.getShooting().position, BlueCloseCoordinatesMeepMeep.getShooting().heading)
                .build());

        meepMeep.setBackground(MeepMeep.Background.FIELD_DECODE_JUICE_BLACK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}