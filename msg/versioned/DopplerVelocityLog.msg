# DVL (Doppler Velocity Log) message
# This message contains the data from a DVL sensor, which is used to measure the velocity
# of a vehicle relative to the seafloor or water column.

uint64 timestamp		# time since system start (microseconds)

float32 altitude

float32[3] velocity     	   # Velocity along x, y, z

float32 smg                         # Speed Made Good, the speed of the vehicle relative to the solution (m/s)

bool velocity_valid                 # Is the velocity value valid

float32 fom                         # Figure of merit, a measure of the accuracy of the velocities (m/s)

bool water_mode_supported           # If the water mode is supported

bool is_water_tracking_enabled      # If the water mode is enabled

bool[4] transducer_beam_validity    # The beam validity for each of the transducers

bool is_dvl_beam_valid              # 3 of the 4 beams have to be valid for this to be true

bool is_velocity_realistic          # Velocity values are within physical limits

float32[3] covariance               # covariance readings of velocity
