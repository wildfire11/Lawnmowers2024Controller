package org.firstinspires.ftc.teamcode;

public class ButtonDebouncer {
    private boolean previousState = false;

    public boolean getDebounced(boolean currentState) {
        if (currentState && !previousState) {
            // Button was pressed, but was not previously pressed
            previousState = true;  // Set flag to indicate that the button has been pressed
            return true;           // Trigger the action once
        } else if (!currentState) {
            // Button is released, reset the previous state
            previousState = false; // Allow future press detection
        }
        return false;              // No action triggered
    }
}
