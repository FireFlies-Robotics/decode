package Red;

import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

import MeepMeep.coordinates.BlueCoordinates.BlueFarCoordinatesMeepMeep;
import MeepMeep.coordinates.RedCoordinatesMeepMeep.RedFarCooddinatesMeepMeep;

public class RedFar {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(670);


        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .build();

        myBot.runAction(myBot.getDrive().actionBuilder(RedFarCooddinatesMeepMeep.getStart())
                .strafeToLinearHeading(RedFarCooddinatesMeepMeep.getShooting().position, RedFarCooddinatesMeepMeep.getShooting().heading)
                        .waitSeconds(1)
                .setTangent(Math.toRadians(-270))
                .splineToLinearHeading(RedFarCooddinatesMeepMeep.getFirstIntakeStart(), RedFarCooddinatesMeepMeep.getFirstIntakeStart().heading)
                .strafeToSplineHeading(RedFarCooddinatesMeepMeep.getFirstIntakeEnd().position, RedFarCooddinatesMeepMeep.getFirstIntakeEnd().heading)
                .strafeToLinearHeading(RedFarCooddinatesMeepMeep.getShooting().position, RedFarCooddinatesMeepMeep.getShooting().heading)
                .setTangent(Math.toRadians(-270))

                .splineToLinearHeading(RedFarCooddinatesMeepMeep.getSecondIntakeStart(), RedFarCooddinatesMeepMeep.getFirstIntakeStart().heading)
                .strafeToSplineHeading(RedFarCooddinatesMeepMeep.getSecondIntakeEnd().position, RedFarCooddinatesMeepMeep.getSecondIntakeEnd().heading)

                .strafeToLinearHeading(RedFarCooddinatesMeepMeep.getShooting().position, RedFarCooddinatesMeepMeep.getShooting().heading)

                .build());

        meepMeep.setBackground(MeepMeep.Background.FIELD_DECODE_OFFICIAL)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}