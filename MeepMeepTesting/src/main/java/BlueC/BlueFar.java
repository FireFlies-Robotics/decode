package BlueC;

import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

import MeepMeep.coordinates.BlueCoordinates.BlueCloseCoordinatesMeepMeep;
import MeepMeep.coordinates.BlueCoordinates.BlueFarCoordinatesMeepMeep;

public class BlueFar {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(670);


        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .build();

        myBot.runAction(myBot.getDrive().actionBuilder(BlueFarCoordinatesMeepMeep.getStart())
                        .splineTo(BlueFarCoordinatesMeepMeep.getShooting().position, BlueFarCoordinatesMeepMeep.getShooting().heading)


                .build());

        meepMeep.setBackground(MeepMeep.Background.FIELD_DECODE_OFFICIAL)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}