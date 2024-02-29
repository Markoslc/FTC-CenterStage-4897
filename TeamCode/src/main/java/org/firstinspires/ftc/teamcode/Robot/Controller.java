package org.firstinspires.ftc.teamcode.Robot;

import static org.firstinspires.ftc.teamcode.Robot.RobotParameters.GamepadButtons;
import static org.firstinspires.ftc.teamcode.Robot.RobotParameters.GamepadSticks;
import static org.firstinspires.ftc.teamcode.Robot.RobotParameters.GamepadTriggers;
import static org.firstinspires.ftc.teamcode.Robot.RobotParameters.L_STICK_X_REVERSED;
import static org.firstinspires.ftc.teamcode.Robot.RobotParameters.L_STICK_Y_REVERSED;
import static org.firstinspires.ftc.teamcode.Robot.RobotParameters.R_STICK_X_REVERSED;
import static org.firstinspires.ftc.teamcode.Robot.RobotParameters.R_STICK_Y_REVERSED;

import androidx.annotation.Nullable;

import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

public class Controller {
    public Button    a;
    public Button    b;
    public Button    x;
    public Button    y;
    public Button    dpadUp;
    public Button    dpadDown;
    public Button    dpadLeft;
    public Button    dpadRight;
    public Button    leftBumper;
    public Button    rightBumper;
    public Button    back;
    public Button    start;
    public Button    guide;
    public Button    leftStickButton;
    public Button    rightStickButton;
    public Trigger   leftTrigger;
    public Trigger   rightTrigger;
    public Stick     leftStick;
    public Stick     rightStick;
    public Button[]  buttons;
    public Trigger[] triggers;
    public Stick[]   sticks;

    /**
     * Constructor for the class Controller that initializes the buttons, triggers, and sticks.
     *
     * @param gamepad the gamepad that the controller is associated with
     */
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

    /**
     * Updates the inputs of the controller
     */
    public void updateInputs() {
        for (Button button : buttons) button.updateButton();
        for (Trigger trigger : triggers) trigger.updateTrigger();
        for (Stick stick : sticks) stick.stickUpdate();
    }

    public static class Button {
        GamepadButtons button;
        Gamepad        gamepad;
        boolean        wasPressed;
        boolean        pressed;

        /**
         * Constructor for the class Button that initializes the button and gamepad.
         *
         * @param button  the button that the class is associated with
         * @param gamepad the gamepad that the button is associated with
         */
        public Button(GamepadButtons button, Gamepad gamepad) {
            this.button = button;
            this.gamepad = gamepad;
            wasPressed = false;
            pressed = false;
        }

        /**
         * Updates the button
         */
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

        /**
         * Returns whether the button is pressed
         *
         * @return A boolean
         */
        public boolean pressed() {
            return pressed;
        }

        /**
         * Returns whether the button was single pressed
         *
         * @return A boolean
         */
        public boolean singlePress() {
            return !wasPressed && pressed;
        }

        /**
         * Returns whether the button was released
         *
         * @return A boolean
         */
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

        /**
         * Constructor for the class Trigger that initializes the trigger and gamepad.
         *
         * @param trigger the trigger that the class is associated with
         * @param gamepad the gamepad that the trigger is associated with
         */
        public Trigger(GamepadTriggers trigger, Gamepad gamepad) {
            this.trigger = trigger;
            this.gamepad = gamepad;
            triggerPress = 0;
            wasPressed = false;
            pressed = false;
        }

        /**
         * Updates the trigger
         */
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

        /**
         * Returns whether the trigger is pressed
         *
         * @return A boolean
         */
        public boolean pressed() {
            return pressed;
        }

        /**
         * Returns whether the trigger was single pressed
         *
         * @return A boolean
         */
        public boolean singlePress() {
            return !wasPressed && pressed;
        }

        /**
         * Returns whether the trigger was released
         *
         * @return A boolean
         */
        public boolean onRelease() {
            return wasPressed && !pressed;
        }

        /**
         * On a scale of 0 to 1, returns how much the trigger is pressed
         *
         * @return A double from 0 to 1
         */
        public double getPress() {
            return triggerPress;
        }
    }

    public static class Stick {
        private final GamepadSticks stick;
        private final Gamepad       gamepad;
        private       double        x;
        private       double        y;

        /**
         * Constructor for the class Stick that initializes the stick and gamepad.
         *
         * @param stick   the stick that the class is associated with
         * @param gamepad the gamepad that the stick is associated with
         */
        public Stick(GamepadSticks stick, Gamepad gamepad) {
            this.stick = stick;
            this.gamepad = gamepad;
            x = 0;
            y = 0;
        }

        /**
         * Updates the stick
         */
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

        /**
         * On a scale of -1 to 1, returns the x value of the stick
         *
         * @return A double from -1 to 1
         */
        public double getX() {
            switch (stick) {
                case L_STICK:
                    return L_STICK_X_REVERSED ? -gamepad.left_stick_x : gamepad.left_stick_x;
                case R_STICK:
                    return R_STICK_X_REVERSED ? -gamepad.right_stick_x : gamepad.right_stick_x;
                default:
                    return 0;
            }
        }

        /**
         * On a scale of -1 to 1, returns the y value of the stick
         *
         * @return A double from -1 to 1
         */
        public double getY() {
            switch (stick) {
                case L_STICK:
                    return L_STICK_Y_REVERSED ? -gamepad.left_stick_y : gamepad.left_stick_y;
                case R_STICK:
                    return R_STICK_Y_REVERSED ? -gamepad.right_stick_y : gamepad.right_stick_y;
                default:
                    return 0;
            }
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
