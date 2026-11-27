package org.firstinspires.ftc.teamcode.automations;

import com.seattlesolvers.solverslib.util.InterpLUT;

public class ShooterCalculations {
    private static final InterpLUT angleILUT = new InterpLUT();
    private static final InterpLUT speedILUT = new InterpLUT();

    public static double getSpeed(double dist) {
        return speedILUT.get(dist);
    }

    public static double getAngle(double dist) {
        return angleILUT.get(dist);
    }

    static {
        // X MUST BE INCREASING ORDER WHEN ADDING
        // otherwise it literally crashes the entire robot with no error message so dont do it

        // bruh
        angleILUT.add(-500, 0);
        speedILUT.add(-500, 1250);

        angleILUT.add(30, 0);
        speedILUT.add(30, 1250);

        // good ones
        angleILUT.add(40, 104);
        speedILUT.add(40, 1350);

        angleILUT.add(50, 175);
        speedILUT.add(50, 1450);

        angleILUT.add(60, 195);
        speedILUT.add(60, 1550);

        angleILUT.add(67, 215);
        speedILUT.add(67, 1550);

        angleILUT.add(70, 215);
        speedILUT.add(70, 1600); // todo slightly lower

        angleILUT.add(80, 215);
        speedILUT.add(80, 1650);

        // switch to 0.8 transfer

        angleILUT.add(90, 235);
        speedILUT.add(90, 1750);

        angleILUT.add(100, 225);
        speedILUT.add(100, 1750);

        angleILUT.add(148, 255);
        speedILUT.add(148, 2100);

        angleILUT.add(157, 255);
        speedILUT.add(157, 2100);

        // ???
        angleILUT.add(500, 225);
        speedILUT.add(500, 2100);

        speedILUT.createLUT();
        angleILUT.createLUT();
    }
}
