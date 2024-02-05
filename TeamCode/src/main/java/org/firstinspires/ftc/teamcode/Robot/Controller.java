package org.firstinspires.ftc.teamcode.Robot;

import com.qualcomm.robotcore.hardware.Gamepad;

import static org.firstinspires.ftc.teamcode.Robot.RobotParameters.*;

import androidx.annotation.Nullable;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

public class Controller {
    public Button  a;
    public Button  b;
    public Button  x;
    public Button  y;
    public Button  dpadUp;
    public Button  dpadDown;
    public Button  dpadLeft;
    public Button  dpadRight;
    public Button  leftBumper;
    public Button  rightBumper;
    public Button  back;
    public Button  start;
    public Button  guide;
    public Button  leftStickButton;
    public Button  rightStickButton;
    public Trigger leftTrigger;
    public Trigger rightTrigger;
    public Stick   leftStick;
    public Stick   rightStick;
    public Button[] buttons;
    public Trigger[] triggers;
    public Stick[] sticks;


    public Controller(Gamepad gamepad) {
        a = new Button(GamepadButtons.A, gamepad);
        b = new Button(GamepadButtons.B, gamepad);
        x = new Button(GamepadButtons.X, gamepad);
        y = new Button(GamepadButtons.Y, gamepad);
        dpadUp = new Button(GamepadButtons.DPAD_UP, gamepad);
        dpadDown = new Button(GamepadButtons.DPAD_DOWN, gamepad);
        dpadLeft = new Button(GamepadButtons.DPAD_LEFT, gamepad);
        dpadRight = new Button(GamepadButtons.DPAD_RIGHT, gamepad);
        leftBumper = new Button(GamepadButtons.L_BUMPER, gamepad);
        rightBumper = new Button(GamepadButtons.R_BUMPER, gamepad);
        back = new Button(GamepadButtons.BACK, gamepad);
        start = new Button(GamepadButtons.START, gamepad);
        guide = new Button(GamepadButtons.GUIDE, gamepad);
        leftStickButton = new Button(GamepadButtons.L_STICK_BUTTON, gamepad);
        rightStickButton = new Button(GamepadButtons.R_STICK_BUTTON, gamepad);
        leftTrigger = new Trigger(GamepadTriggers.L_TRIGGER, gamepad);
        rightTrigger = new Trigger(GamepadTriggers.R_TRIGGER, gamepad);
        leftStick = new Stick(GamepadSticks.L_STICK, gamepad);
        rightStick = new Stick(GamepadSticks.R_STICK, gamepad);

        buttons = new Button[]{a, b, x, y, dpadUp, dpadDown, dpadLeft, dpadRight,
                leftBumper, rightBumper,
                back, start, guide,
                leftStickButton, rightStickButton};
        triggers = new Trigger[]{leftTrigger, rightTrigger};
        sticks = new Stick[]{leftStick, rightStick};
    }

    public void updateInputs(){
        for(Button button : buttons) button.updateButton();
        for (Trigger trigger : triggers) trigger.updateTrigger();
        for (Stick stick : sticks) stick.stickUpdate();
    }

    public static class Button {
        GamepadButtons button;
        Gamepad        gamepad;
        boolean        wasPressed;
        boolean        pressed;

        public Button(GamepadButtons button, Gamepad gamepad) {
            this.button = button;
            this.gamepad = gamepad;
            wasPressed = false;
            pressed = false;
        }

        private void updateButton() {
            wasPressed = pressed;

            switch (button) {
                case A:
                    pressed = gamepad.a;
                    break;
                case B:
                    pressed = gamepad.b;
                    break;
                case X:
                    pressed = gamepad.x;
                    break;
                case Y:
                    pressed = gamepad.y;
                    break;
                case DPAD_UP:
                    pressed = gamepad.dpad_up;
                    break;
                case DPAD_DOWN:
                    pressed = gamepad.dpad_down;
                    break;
                case DPAD_LEFT:
                    pressed = gamepad.dpad_left;
                    break;
                case DPAD_RIGHT:
                    pressed = gamepad.dpad_right;
                    break;
                case L_BUMPER:
                    pressed = gamepad.left_bumper;
                    break;
                case R_BUMPER:
                    pressed = gamepad.right_bumper;
                    break;
                case BACK:
                    pressed = gamepad.back;
                    break;
                case START:
                    pressed = gamepad.start;
                    break;
                case GUIDE:
                    pressed = gamepad.guide;
                    break;
                case L_STICK_BUTTON:
                    pressed = gamepad.left_stick_button;
                    break;
                case R_STICK_BUTTON:
                    pressed = gamepad.right_stick_button;
                    break;
            }
        }

        public boolean pressed() {
            return pressed;
        }

        public boolean singlePress() {
            return !wasPressed && pressed;
        }

        public boolean onRelease() {
            return wasPressed && !pressed;
        }
    }

    public static class Trigger {
        private final GamepadTriggers trigger;
        private final Gamepad         gamepad;
        private       double          triggerPress;
        private       boolean         wasPressed;
        private       boolean         pressed;

        public Trigger(GamepadTriggers trigger, Gamepad gamepad) {
            this.trigger = trigger;
            this.gamepad = gamepad;
            triggerPress = 0;
            wasPressed = false;
            pressed = false;
        }

        private void updateTrigger() {
            wasPressed = pressed;
            switch (trigger) {
                case L_TRIGGER:
                    triggerPress = gamepad.left_trigger;
                    pressed = triggerPress > 0;
                    break;
                case R_TRIGGER:
                    triggerPress = gamepad.right_trigger;
                    pressed = triggerPress > 0;
                    break;
            }
        }

        public boolean pressed() {
            return pressed;
        }

        public boolean singlePress() {
            return !wasPressed && pressed;
        }

        public boolean onRelease() {
            return wasPressed && !pressed;
        }

        public double getPress() {
            return triggerPress;
        }
    }

    public static class Stick {
        private final GamepadSticks stick;
        private final Gamepad       gamepad;
        private       double        x;
        private       double        y;

        public Stick(GamepadSticks stick, Gamepad gamepad) {
            this.stick = stick;
            this.gamepad = gamepad;
            x = 0;
            y = 0;
        }

        private void stickUpdate() {
            switch (stick) {
                case L_STICK:
                    x = gamepad.left_stick_x;
                    y = gamepad.left_stick_y;
                    break;
                case R_STICK:
                    x = gamepad.right_stick_x;
                    y = gamepad.right_stick_y;
            }
        }

        public double getX() {
            return STICK_X_REVERSED ? -gamepad.left_stick_x : gamepad.left_stick_x;
        }

        public double getY() {
            return STICK_Y_REVERSED ? -gamepad.left_stick_y : gamepad.left_stick_y;
        }

        /**
         * Returns the magnitude of the stick
         *
         * @return A double
         */
        public double getMagnitude() {
            return Math.sqrt(Math.pow(x, 2) + Math.pow(y, 2));
        }

        /**
         * Returns the angle formed by the stick
         *
         * @param unit The unit that the angle will be returned in. Leave blank for radians
         * @return A double
         */
        public double getAngle(@Nullable AngleUnit... unit) {
            stickUpdate();
            if (unit != null && unit.length > 0) {
                switch (unit[0]) {
                    case DEGREES:
                        return Math.atan2(getX(), getY()) * 180 / Math.PI;
                    case RADIANS:
                        return Math.atan2(getX(), getY());
                }
            }
            return Math.atan2(getX(), getY());
        }
    }
}
