# Configuration variables for the show drone prototype sent to us by Bitcraze
# in Dec 2020

# Mass of the show drone prototype is about 100g
CF_MODEL_MASS=0.1f

# Use UWB positioning and sqrt position controller
CF_MODEL_POSITIONING=uwb
POSITION_CONTROLLER=sqrt

# Configure ESC and start disarmed; use some idle thrust as well
CFLAGS += -DENABLE_ONESHOT125 -DSTART_DISARMED -DDEFAULT_IDLE_THRUST=5000

# Specify the desired PWM ratio for motor tests
CFLAGS += -DDEFAULT_PROP_TEST_PWM_RATIO=10000

## The low voltage thresholds have to be tuned here, these are simply obtained
## by multiplying whatever we've had with 2 since the new show drone prototype
## used two cells.
CFLAGS += -DLOW_VOLTAGE=6.4f -DCRITICAL_LOW_VOLTAGE=6.0f
CFLAGS += -DPREFLIGHT_MIN_BATTERY_VOLTAGE=7.2f
