package org.firstinspires.ftc.teamcode.LED.LED;

import static org.firstinspires.ftc.teamcode.LED.LED.Prism.GoBildaPrismDriver.LayerHeight;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.LED.LED.Prism.Color;
import org.firstinspires.ftc.teamcode.LED.LED.Prism.GoBildaPrismDriver;
import org.firstinspires.ftc.teamcode.LED.LED.Prism.PrismAnimations;
import org.firstinspires.ftc.teamcode.LED.LED.Prism.PrismAnimations.AnimationType;
import org.firstinspires.ftc.teamcode.LED.LED.Prism.PrismAnimations.PoliceLights;
import org.firstinspires.ftc.teamcode.LED.LED.Prism.GoBildaPrismDriver.Artboard;

import java.util.concurrent.TimeUnit;
@TeleOp(name="Prism Configurator", group="Linear OpMode")
//@Disabled

public class LEDAdjuster extends LinearOpMode {
    GoBildaPrismDriver prism;

    // an enum which is used in a mini state machine to allow the user to set different colors.
    public enum AnimationColor {
        PRIMARY_COLOR,
        SECONDARY_COLOR,
        TERTIARY_COLOR,
    }

    AnimationColor animationColor = AnimationColor.PRIMARY_COLOR;

    // Set a default style for the Police Lights Animation.
    PoliceLights.PoliceLightsStyle policeLightsStyle = PoliceLights.PoliceLightsStyle.Style1;

    int startPoint = 0; // the start LED for any configured animation
    int endPoint = 35; // the end LED for a configured animation
    int brightness = 50; // the brightness of configured animation
    int period = 1000; // the period of a configured animation
    float speed = 0.5F; // the speed of a configured animation

    int layerSelector = 0; // an integer used to create a cursor to select a layer
    int animationSelector = 1; // animation cursor
    int artboardSelector = 0; // artboard cursor

    String hsbTelemetry; // string meant to send over telemetry that gets updated by hsbViaJoystick()
    String hueTelemetry; // updated by hueViaJoystick()

    /*
     * An array of colors passed to the SingleFill animation.
     */
    Color[] singleFillColors = {
            Color.RED, Color.WHITE, Color.BLUE
    };

    AnimationType selectedAnimation = AnimationType.SOLID; // store the animation that is being selected.

    /*
     * the enum powering the main state machine that this code moves through, each state represents
     * a page the user can see displayed via telemetry on the driver's station.
     */
    public enum ConfigState {
        WELCOME_SCREEN,
        SELECT_LAYER,
        SET_ENDPOINTS,
        SELECT_ANIMATION,
        CONFIGURE_ANIMATION,
        SET_BRIGHTNESS,
        SET_SPEED,
        FORK_IN_THE_ROAD,
        SAVE_TO_ARTBOARD,
        COMPLETE;
    }

    ConfigState configState = ConfigState.WELCOME_SCREEN;

    /*
     * This is actually a bit of a duplicate of the LayerHeight found in PrismAnimations. This adds
     * an animation slot which can be stored at each position in the enum. This is not required for
     * the Prism side, but I want to be able to show a user what animation is stored at what layer
     * after they've created their first animation.
     */
    public enum Layers {
        LAYER_0(AnimationType.NONE, 0, LayerHeight.LAYER_0),
        LAYER_1(AnimationType.NONE, 1, LayerHeight.LAYER_1),
        LAYER_2(AnimationType.NONE, 2, LayerHeight.LAYER_2),
        LAYER_3(AnimationType.NONE, 3, LayerHeight.LAYER_3),
        LAYER_4(AnimationType.NONE, 4, LayerHeight.LAYER_4),
        LAYER_5(AnimationType.NONE, 5, LayerHeight.LAYER_5),
        LAYER_6(AnimationType.NONE, 6, LayerHeight.LAYER_6),
        LAYER_7(AnimationType.NONE, 7, LayerHeight.LAYER_7),
        LAYER_8(AnimationType.NONE, 8, LayerHeight.LAYER_8),
        LAYER_9(AnimationType.NONE, 9, LayerHeight.LAYER_9);

        private AnimationType animationType;
        private final int index;
        private final LayerHeight layerHeight;

        Layers(AnimationType animationType, int index, LayerHeight layerHeight) {
            this.animationType = animationType;
            this.index = index;
            this.layerHeight = layerHeight;
        }
    }

    /*
     * This enum captures the kind of speed we can control on the animation.
     */
    public enum SpeedType {
        NO_SPEED,
        PERIOD_ONLY,
        SPEED_ONLY,
        PERIOD_AND_SPEED,
    }

    Layers selectedLayer = Layers.LAYER_0;

    Artboard selectedArtboard = Artboard.ARTBOARD_0;

    /*
     * Create each Prism Animation which can be customized by the user.
     */
    PrismAnimations.Solid solid = new PrismAnimations.Solid();
    PrismAnimations.Solid endpointsAnimation = new PrismAnimations.Solid();
    PrismAnimations.Blink blink = new PrismAnimations.Blink();
    PrismAnimations.Pulse pulse = new PrismAnimations.Pulse();
    PrismAnimations.SineWave sineWave = new PrismAnimations.SineWave();
    PrismAnimations.DroidScan droidScan = new PrismAnimations.DroidScan();
    PrismAnimations.Rainbow rainbow = new PrismAnimations.Rainbow();
    PrismAnimations.Snakes snakes = new PrismAnimations.Snakes();
    PrismAnimations.Random random = new PrismAnimations.Random();
    PrismAnimations.Sparkle sparkle = new PrismAnimations.Sparkle();
    PrismAnimations.SingleFill singleFill = new PrismAnimations.SingleFill();
    PrismAnimations.RainbowSnakes rainbowSnakes = new PrismAnimations.RainbowSnakes();
    PoliceLights policeLights = new PoliceLights();


    @Override
    public void runOpMode() {
        prism = hardwareMap.get(GoBildaPrismDriver.class, "prism");

        telemetry.addLine("Welcome to the Prism Configurator, enjoy these fun stats " +
                "and click the 'Play' button to continue");
        telemetry.addLine("");

        telemetry.addData("Device ID", prism.getDeviceID());
        telemetry.addLine(prism.getFirmwareVersionString());
        telemetry.addLine(prism.getHardwareVersionString());
        telemetry.addData("Power Cycle Count", prism.getPowerCycleCount());
        telemetry.addData("Run Time (Minutes)", prism.getRunTime(TimeUnit.MINUTES));
        telemetry.addData("Run Time (Hours)", prism.getRunTime(TimeUnit.HOURS));
        telemetry.update();

        // Wait for the game to start (driver presses START)
        waitForStart();
        resetRuntime();
        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            switch (configState) {
                case SELECT_LAYER:
                    /*
                     * You can't convince me that this is the correct way to implement a cursor,
                     * but you also can't tell me this one doesn't work.
                     * We allow the user to increment the layerSelecter variable, before constraining
                     * it to within a valid range. We then pass that into our "selectLayer()"
                     * function which displays the UI elements via telemetry and saves the selected
                     * layer every loop.
                     */
                    if (gamepad1.dpadUpWasPressed()) {
                        layerSelector -= 1;
                    }
                    if (gamepad1.dpadDownWasPressed()) {
                        layerSelector += 1;
                    }
                    layerSelector = Math.min(9, Math.max(0, layerSelector));
                    selectLayer(layerSelector);
                    if (gamepad1.aWasPressed()) {
                        configureEndpointsAnimation(true);
                        configState = ConfigState.SET_ENDPOINTS;
                    }
                    break;
            }
            telemetry.update();
            sleep(20);
        }
    }

    public void selectLayer(int selector) {
        telemetry.addLine("Select the Layer that you wish to save an Animation to.");
        telemetry.addLine("Use D-Pad up and D-Pad down to navigate through the layers.");
        telemetry.addLine("");
        telemetry.addData("LAYER_0 Animation", Layers.LAYER_0.animationType + layerCursor(Layers.LAYER_0, selector));
        telemetry.addData("LAYER_1 Animation", Layers.LAYER_1.animationType + layerCursor(Layers.LAYER_1, selector));
        telemetry.addData("LAYER_2 Animation", Layers.LAYER_2.animationType + layerCursor(Layers.LAYER_2, selector));
        telemetry.addData("LAYER_3 Animation", Layers.LAYER_3.animationType + layerCursor(Layers.LAYER_3, selector));
        telemetry.addData("LAYER_4 Animation", Layers.LAYER_4.animationType + layerCursor(Layers.LAYER_4, selector));
        telemetry.addData("LAYER_5 Animation", Layers.LAYER_5.animationType + layerCursor(Layers.LAYER_5, selector));
        telemetry.addData("LAYER_6 Animation", Layers.LAYER_6.animationType + layerCursor(Layers.LAYER_6, selector));
        telemetry.addData("LAYER_7 Animation", Layers.LAYER_7.animationType + layerCursor(Layers.LAYER_7, selector));
        telemetry.addData("LAYER_8 Animation", Layers.LAYER_8.animationType + layerCursor(Layers.LAYER_8, selector));
        telemetry.addData("LAYER_9 Animation", Layers.LAYER_9.animationType + layerCursor(Layers.LAYER_9, selector));
        telemetry.addLine("");
        telemetry.addLine("Press A to continue");
    }
    public void configureEndpointsAnimation(boolean isBeingInserted) {
        endpointsAnimation.setStartIndex(startPoint);
        endpointsAnimation.setStopIndex(endPoint);
        if (isBeingInserted) {
            prism.insertAndUpdateAnimation(selectedLayer.layerHeight, endpointsAnimation);
        } else {
            prism.updateAnimationFromIndex(selectedLayer.layerHeight);
        }
    }
    public String layerCursor(Layers layer, int selector){
        if(layer.index == selector){
            selectedLayer = layer;
            return "<--";
        }else return "";
    }
}

