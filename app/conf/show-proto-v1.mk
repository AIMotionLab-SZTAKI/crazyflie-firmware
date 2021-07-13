# Configuration variables for the show drone prototype sent to us by Bitcraze
# in Dec 2020

# Mass of the show drone prototype is about 100g
CF_MODEL_MASS=0.1f

# Use UWB positioning
CF_MODEL_POSITIONING=uwb

# Configure ESC and start disarmed; use some idle thrust as well
CFLAGS += -DENABLE_ONESHOT125 -DSTART_DISARMED -DDEFAULT_IDLE_THRUST=5000

## The low voltage thresholds have to be tuned here, these are simply obtained
## by multiplying whatever we've had with 2 since the new show drone prototype
## used two cells.
CFLAGS += -DLOW_VOLTAGE=6.4f -DCRITICAL_LOW_VOLTAGE=6.0f
