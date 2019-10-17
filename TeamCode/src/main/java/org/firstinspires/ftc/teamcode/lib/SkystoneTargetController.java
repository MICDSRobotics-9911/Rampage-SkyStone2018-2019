package org.firstinspires.ftc.teamcode.lib;

import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;

import java.util.List;

/**
 * Manages all skystone targets. <b>Is to be called in pre-init phase of autonomous</b>
 * @since 10/14/19
 * @see VuforiaTrackables
 */
public class SkystoneTargetController {

    // all trackables, as Vuforia sees them
    private VuforiaTrackables trackables;

    // all trackables, as a List sees them
    private List<VuforiaTrackable> listTrackables;

    // all trackables as individual variables
    private VuforiaTrackable RedPerimiterTarget2;
    private VuforiaTrackable RedPerimiterTarget1;
    private VuforiaTrackable RearPerimeterTarget2;
    private VuforiaTrackable RearPerimeterTarget1;
    private VuforiaTrackable FrontPerimeterTarget2;
    private VuforiaTrackable FrontPerimieterTarget1;
    private VuforiaTrackable BluePerimeterTarget2;
    private VuforiaTrackable BluePerimeterTarget1;

    public SkystoneTargetController(VuforiaLocalizer vuforiaLocalizer) {
        this.trackables = vuforiaLocalizer.loadTrackablesFromAsset("SkyStone2");

        this.RedPerimiterTarget2 = this.trackables.get(0);
        this.RedPerimiterTarget2.setName("RedPerimiterTarget1");

        this.RedPerimiterTarget1 = this.trackables.get(1);
        this.RedPerimiterTarget1.setName("RedPerimiterTarget1");

        this.RearPerimeterTarget2 = this.trackables.get(2);
        this.RearPerimeterTarget2.setName("RearPerimeterTarget2");

        this.RearPerimeterTarget1 = this.trackables.get(3);
        this.RearPerimeterTarget1.setName("RearPerimeterTarget1");

        this.FrontPerimeterTarget2 = this.trackables.get(4);
        this.FrontPerimeterTarget2.setName("FrontPerimeterTarget2");

        this.FrontPerimieterTarget1 = this.trackables.get(5);
        this.FrontPerimieterTarget1.setName("FrontPerimieterTarget1");

        this.BluePerimeterTarget2 = this.trackables.get(6);
        this.BluePerimeterTarget2.setName("BluePerimeterTarget2");

        this.BluePerimeterTarget1 = this.trackables.get(7);
        this.BluePerimeterTarget1.setName("BluePerimeterTarget1");

        this.listTrackables.addAll(this.trackables);
    }

    public VuforiaTrackables getTrackables() {
        return trackables;
    }

    public List<VuforiaTrackable> getListTrackables() {
        return listTrackables;
    }

    public VuforiaTrackable getRedPerimiterTarget2() {
        return RedPerimiterTarget2;
    }

    public VuforiaTrackable getRedPerimiterTarget1() {
        return RedPerimiterTarget1;
    }

    public VuforiaTrackable getRearPerimeterTarget2() {
        return RearPerimeterTarget2;
    }

    public VuforiaTrackable getRearPerimeterTarget1() {
        return RearPerimeterTarget1;
    }

    public VuforiaTrackable getFrontPerimeterTarget2() {
        return FrontPerimeterTarget2;
    }

    public VuforiaTrackable getFrontPerimieterTarget1() {
        return FrontPerimieterTarget1;
    }

    public VuforiaTrackable getBluePerimeterTarget2() {
        return BluePerimeterTarget2;
    }

    public VuforiaTrackable getBluePerimeterTarget1() {
        return BluePerimeterTarget1;
    }
}
