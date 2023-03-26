package frc.robot.Custom;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.button.Button;

@SuppressWarnings("deprecation")
public class DPadButton extends Button {

    static Joystick joystick;
    static Direction direction;

    public DPadButton(Joystick Joystick, Direction Direction) {
        super(() -> get());
        joystick = Joystick;
        direction = Direction;
    }

    public static enum Direction {
        UP(0), RIGHT(90), DOWN(180), LEFT(270);

        int direction;

        private Direction(int direction) {
            this.direction = direction;
        }
    }

    public static boolean get() {
        int dPadValue = joystick.getPOV();
        return (dPadValue == direction.direction) || (dPadValue == (direction.direction + 45) % 360)
                || (dPadValue == (direction.direction + 315) % 360);
    }
}