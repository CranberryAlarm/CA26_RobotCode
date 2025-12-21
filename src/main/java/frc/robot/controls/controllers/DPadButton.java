//Credit: https://gist.github.com/pordonj/970b2c189cc6ee06388b3e2f12abcb72

package frc.robot.controls.controllers;

public class DPadButton {

    private FilteredController m_controller;
    private Direction m_direction;

    private boolean m_lock = false;

    public DPadButton(FilteredController controller, Direction direction) {
        this.m_controller = controller;
        this.m_direction = direction;
    }

    public static enum Direction {
        UP(0), RIGHT(90), DOWN(180), LEFT(270);

        int direction;

        private Direction(int direction) {
            this.direction = direction;
        }
    }

    public boolean get() {
        int dPadValue = m_controller.getPOV();
        return dPadValue == m_direction.direction;
    }

    public boolean getPressed() {
        // Rising edge detection
        if(this.get()) {
            if(!m_lock) {
                m_lock = true;
                return true;
            } else {
                return false;
            }
        } else {
            m_lock = false;
            return false;
        }
    }
}