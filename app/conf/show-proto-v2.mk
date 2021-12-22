# Configuration variables for Nina's show drone prototype with Lighthouse
# instead of UWB

# Mass of the show drone prototype is about 100g
CF_MODEL_MASS=0.1f

# Use Lighthouse positioning and the standard PID controller
CF_MODEL_POSITIONING=lighthouse

# Configure ESC
CFLAGS += -DENABLE_ONESHOT125

# Start disarmed; use some idle thrust as well. Enable this if needed.
# CFLAGS += -DSTART_DISARMED -DDEFAULT_IDLE_THRUST=5000

# Specify the desired PWM ratio for motor tests
CFLAGS += -DDEFAULT_PROP_TEST_PWM_RATIO=10000

# Disable detection of connected USB charger because it does not work on this
# board (it always says that the charger is connected)
CFLAGS += -DDISABLE_CHARGER_DETECTION

# Use smoother (less aggressive) takeoff as it is perfectly fine with Lighthouse
# and it looks less scary with a larger drone
CFLAGS += -DSHOW_SMOOTH_TAKEOFF

# Reduce LED ring brightness because the motors and the LED ring together draw
# too much power
CFLAGS += -DLED_RING_REDUCE_BRIGHTNESS=2

