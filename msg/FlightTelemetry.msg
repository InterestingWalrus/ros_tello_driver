std_msgs/Header header

##### Raw String #####

string raw

#### SDK Version #####

uint8 SDK_UNKNOWN = 0
uint8 SDK_1_3 = 1
uint8 SDK_2_0 = 2
uint8 sdk

#### data from SKD 1.3 ####

int32 pitch  # degrees
int32 roll   # degrees
int32 yaw    # degrees relative to starting orientation

int32 vgx    # x velocity m/s
int32 vgy    # y velocity m/s
int32 vgz    # z velocity m/s

int32 ltemp  # lowest temp in C
int32 htemp  # highest temp in C

int32 tof    # time of flight distance in cm 
int32 h      # height in cm


int32 battery # battery %

float32 alti   #barometric altitude in m

int32 time     #time used by motors in seconds

#Acceleration due to gravity mm/s^2 Angular acceleration data (0.001g)
float32 agx   #x forward
float32 agy   #x right
float32 agz   #z down

#######
# SDK 2.0+ fields
#######

int32 mid     # mission pad id, or -1 if not found
int32 x       # x coord relative to mission pad in cm, or 0 if not found
int32 y       # y coord relative to mission pad in cm, or 0 if not found
int32 z       # z coord relative to mission pad in cm, or 0 if not found

##############################EXTRAS############################

bool imu_state
bool pressure_sensor_state
bool down_visual_state
bool power_state
bool battery_state
bool gravity_state
bool wind_state

uint8 electrical_machinery_state

# flymode=1: landed; =6: flying
uint8 fly_mode
float32 throw_takeoff_timer_sec
uint8 camera_state