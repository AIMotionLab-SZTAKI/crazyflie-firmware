# Configuration variables for Nina's show drone prototype with Lighthouse
# instead of UWB

# Mass of the show drone prototype is about 100g
CF_MODEL_MASS=0.1f

# Use Lighthouse positioning
CF_MODEL_POSITIONING=lighthouse

# Configure ESC
CFLAGS += -DENABLE_ONESHOT125

# Start disarmed; use some idle thrust as well. Enable this if needed.
# CFLAGS += -DSTART_DISARMED -DDEFAULT_IDLE_THRUST=5000

