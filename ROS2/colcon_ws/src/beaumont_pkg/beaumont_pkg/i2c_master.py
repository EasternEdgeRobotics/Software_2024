import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

from eer_messages.msg import ThrusterMultipliers, PilotInput, DiagnosticsData, OutsideTempProbeData
from eer_messages.action import AutoMode 
from eer_messages.srv import HSVColours
from std_msgs.msg import String

from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor

import board
from smbus2 import SMBus
from adafruit_bno055 import BNO055_I2C
from math import sqrt


# Thurster channels are based on Beaumont
THRUSTER_CHANNELS = {
    "for-port-top": 1, 
    "for-star-top": 0, 
    "aft-port-top": 4,
    "aft-star-top": 3,
    "for-port-bot": 2,
    "for-star-bot": 6,
    "aft-port-bot": 5,
    "aft-star-bot": 7
}

# In the thruster class, the target speed is set by the user. Each thruster accelerates towards the target speed by the acceleration below
THRUSTER_ACCELERATION = 5

# Determines how often each thruster accelerates towards the target speed set by the user
# THRUSTER_TICK_PERIOD = 0.05

RP2040_ADDRESS = 0x08

STM32_ADDRESS = 0x69 

ADC_ADDRESSES = {
    "adc_48v_bus": 0x55,
    "adc_12v_bus":0x56,
    "adc_5v_bus":0x59
}

ADC_VOLTAGE_DIVIDER_VALUES = {
    "adc_48v_bus": (20000,1000),
    "adc_12v_bus":(5000,1000),
    "adc_5v_bus":(2000,1000)
}

TEMPERATURE_SENSOR_ADDRESSES = {
    "power_board_u8":0x48,
    "power_board_u9":0x49,
    "power_board_u10":0x4a,
    "mega_board_ic2":0x4b,
    "power_board_u11":0x4c,
    "mega_board_ic1":0x4e,
}

# How often to read data on i2c bus
DIAGNOSTICS_REQUESTS_PERIOD = 2

class Thruster:
    """Thruster class."""
    def __init__(self, bus, thruster_position):
        """
        Setup the thruster or servo.

        :param bus: SMBus object to communicate on I2C bus
        :param thruster position: The position of the thruster as a string 
        """
        self.bus = bus
        self.thruster_position = thruster_position

        self.thruster_armed = False

        # 127 corresponds to the middle of the range (0-254), corresponding to duty cycles of (1000-2000), wher 1500 is center speed (no rotation)
        self.target = 127
        self.current = 127

        # Arm thruster to center speed on initialization
        self.arm_thruster()

    def arm_thruster(self):
        
        # SMBus throws an OSError if it fails to communicate with RP2040
        try: 
            self.bus.write_byte_data(RP2040_ADDRESS, THRUSTER_CHANNELS[self.thruster_position],127)
            self.thruster_armed = True 
        except OSError:
            pass

    def fly(self, speed):
        """
        Drive thrusters using given speed parameters.

        :param speed: speed of thrusters with valid range between -1.0 to 1.0
        """
        self.target = speed*127 + 127

    def tick(self):
        
        if not self.thruster_armed:
            self.arm_thruster()
            return

        if(self.current != self.target): # Must accelerate towards target speed

            if(abs(self.target - self.current) > THRUSTER_ACCELERATION): 
                
                # Determine the direction in which thruster must accelerate
                direction = (self.target - self.current) / abs(self.target - self.current) 

                # Accelerate towards desired direction by THRUSTER_ACCELERATION
                self.current += int(direction * THRUSTER_ACCELERATION)

                # Ensure the current speed does not go out of range
                if self.current > 254:
                    self.current = 254
                elif self.current < 0:
                    self.current = 0

                # If communication with RP2040 fails, attempt to arm the thruster again as soon as communication is reestablished
                try:
                    self.bus.write_byte_data(RP2040_ADDRESS, THRUSTER_CHANNELS[self.thruster_position], int(self.current)) 
                except OSError:
                    self.thruster_armed = False

            else:
                self.current = self.target

                # If communication with RP2040 fails, attempt to arm the thruster again as soon as communication is reestablished
                try:
                    self.bus.write_byte_data(RP2040_ADDRESS, THRUSTER_CHANNELS[self.thruster_position], int(self.current)) 
                except OSError:
                    self.thruster_armed = False

class I2CMaster(Node):
    '''
    This class will communicate to multiple devices on the same I2C bus. These include the following:

        1. RP2040: Talks to the 8 Thrusters connected to the bot. Communication will be achieved by the SMBUS library using a eer proprietary i2c protocol.
        2. STM32: Talks to claw, stepper motor, dimmable LEDs, and outside temperature probe. Programmed by the electrical team, 
           thus communication will be achieved by the SMBUS library using a eer proprietary i2c protocol
        3. 3 Analog digital converters (ADCs) on the 48v, 12v, and 5v buses to mesure board health. Communication will be achieved with 
           the SMBUS library.
        4. 6 Temperature sensors on the boards (4 on power board, 2 on mega board). Communication will be achieved with the SMBUS library
        5. BNO055 Inertial Mesurement Unit (IMU). Communication will be done using the adafruit adafruit_bno055 library.

    All aforementioned devices will be on the same I2C bus, meaning that it is only possible to communicate to one at a time. This means these processes
    are not thread-safe. Cases 1 and 2 are need-based communications (there will be a button to request outside temperature probe readings), while cases 
    3, 4, and 5 are time based.

    To ensure proper communication, a ROS2 Mutually Exclusive Callback Group along with timers will be used. This will avoid race conditions and writing two requests to 
    the i2c bus at the same time (https://docs.ros.org/en/humble/How-To-Guides/Using-callback-groups.html). Cases 1 and 2 will also be moved to time-based, using the 
    last-recieved input from topsides (it should be ensured that topsides sends pilot input even when no buttons are pressed and no axes are moved). Topsides input comes in at 10Hz.          
    '''

    def __init__(self):
        super().__init__('i2c_master')

        self.debugger_mode = False

        # Subscribers
        self.copilot_listener = self.create_subscription(ThrusterMultipliers, 'thruster_multipliers', self.copilot_listener_callback, 100)
        self.pilot_listener = self.create_subscription(PilotInput, 'controller_input', self.pilot_listener_callback, 1) # input recieved at 10Hz

        # Publishers
        # Comment out code below to remove slowdown
        self.diagnostics_data_publisher = self.create_publisher(DiagnosticsData, "diagnostics", 1)
        self.outside_temperature_probe_data_publisher = self.create_publisher(OutsideTempProbeData, "outside_temp_probe", 1)
        self.autonomous_mode_publisher = self.create_publisher(String, "/autonomous_mode_status", 10)

        # Autonomus movement node
        self._action_client = ActionClient(self, AutoMode, 'autonomus_brain_coral_transplant')
        self.autonomous_mode_active = False

        # Client to fetch the hsv colour values camera saved on the task_manager database
        self.brain_coral_hsv_colour_bounds_client = self.create_client(HSVColours, 'set_color', callback_group=ReentrantCallbackGroup())

        # The default bounds filter for the colour Black
        self.brain_coral_hsv_colour_bounds = {
            "upper_hsv":[10,10,40],
            "lower_hsv":[0,0,0]
        }

        # prevent unused variable warning
        self.copilot_listener 
        self.pilot_listener

        self.headlight_led_brightness = 50
        
        #################################################################
        ################### THRUSTER CALIBRATION MODE ###################
        #################################################################   

        self.thruster_calibration_mode = False

        if self.thruster_calibration_mode:
            self.debugger_mode = True
            self.current_thruster = 0


        ################################################
        ################### DEBUGGER ###################
        ################################################   

        if self.debugger_mode:
            self.debugger = self.create_publisher(String, 'debugger', 10)

        ###############################################################
        ################### INITIALIZE ADAFRUIT I2C ###################
        ###############################################################        

        self.i2c = None

        try:
            self.i2c = board.I2C()
        except:
            self.get_logger().error("No Hardware on I2C bus")


        #################################################
        ###################### IMU ######################
        #################################################

        self.imu_detected = False

        if self.i2c is not None:
            # Connect to BNO055
            try:
                self.imu_sensor = BNO055_I2C(self.i2c)
                self.last_temperature_val = 0xFFFF # per recommendation on (https://learn.adafruit.com/adafruit-bno055-absolute-orientation-sensor/python-circuitpython)

                self.imu_detected = True

                self.get_logger().info("BNO055 DETECTED ON I2C BUS")
            except:
                self.get_logger().error("CANNOT FIND BNO055 ON I2C BUS!")
        

        ##############################################################
        ###################### Initialize SMBUS ######################
        ##############################################################

        self.bus = None

        try:
            self.bus = SMBus(1)
            self.get_logger().info("INITIALIZED SMBUS!")
        except:
            self.get_logger().error("COULD NOT INITIALIZE SMBUS")


        #################################################
        ################### THRUSTERS ###################
        #################################################

        self.power_multiplier = 0.2
        self.surge_multiplier = 0
        self.sway_multiplier = 0
        self.heave_multiplier = 0
        self.pitch_multiplier = 0
        self.yaw_multiplier = 0

        self.connected_channels = {}

        if self.bus is not None:
            for thruster_position in THRUSTER_CHANNELS:  
                self.connected_channels[THRUSTER_CHANNELS[thruster_position]] = Thruster(self.bus, thruster_position)

        ##################################################
        ###################### ADCs ######################
        ##################################################

        if self.bus is not None:

            self.configured_adcs = {}
            
            for key in ADC_ADDRESSES:
                self.configured_adcs[key] = False       
        

    def copilot_listener_callback(self, msg):
        self.power_multiplier = float(msg.power/100)
        self.surge_multiplier = float(msg.surge/100)
        self.sway_multiplier = float(msg.sway/100)
        self.heave_multiplier = float(msg.heave/100)
        self.pitch_multiplier = float(msg.pitch/100)
        self.yaw_multiplier = float(msg.yaw/100)

    def tick_thrusters(self):
        for channel in self.connected_channels:
            self.connected_channels[channel].tick()

    def obtain_imu_data(self, diagnostics_data):
        '''
        Grabs relevant information form the IMU on the I2C Bus.
        Every time a piece of data is accessed, that is an i2c read transation. These transactions finish by the time the value is
        returned. See (https://github.com/adafruit/Adafruit_CircuitPython_BNO055/blob/main/adafruit_bno055.py).
        '''

        diagnostics_data.temperature = self.imu_sensor.temperature

        # This code below was recommended by Adafruit (https://learn.adafruit.com/adafruit-bno055-absolute-orientation-sensor/python-circuitpython)
        if abs(diagnostics_data.temperature - self.last_temperature_val) == 128:
            diagnostics_data.temperature = 0b00111111 & diagnostics_data.temperature
        self.last_temperature_val = diagnostics_data.temperature

        # Below assignments only work because the imu returns size 3 tuples
        # The ROS IMUData message type demands size 3 arrays
        diagnostics_data.acceleration = list(self.imu_sensor.acceleration)
        diagnostics_data.magnetic = list(self.imu_sensor.magnetic) 
        diagnostics_data.euler = list(self.imu_sensor.euler)
        diagnostics_data.linear_acceleration = list(self.imu_sensor.linear_acceleration)

        return diagnostics_data


    def obtain_adc_data(self, diagnostics_data):
        """
        Initially, this function will set ADC configurations. This should be done on power-up as recommended on ADC datasheet.

        In configuration, the 3 MSB set the automatic conversion mode, which configures the ADC to continually perform conversions
        without read requests. Based on the table in the datasheet, the three bits set the cycle time (i.e. sample rate) of 
        these automatic conversions.

        Not configuring the ADC to automatic conversion mode will essentially make it so that the sample rate is the read 
        rate (1Hz). This means every piece of data read will be one second old.
        
        The other bits have to do with how the ADC handles alert conditions (when readings exceed a user-set min or max).
        For our purposes, it is ok to keep them as their default configurations as readings will be displayed on topsides.

        If, at any point, the smbus2 read_byte_data function returns all zeros, it is assumed the adcs are not powered.
        This is because the read_byte_data function does not care for slave acknowledgement and will just read zeros if 
        the slave does not exist. This function will attempt to configure the slaves again the next time it's called.

        The below code does everything needed for our purposes. All ADC registers are listed below:
        0b000 - Conversion Result (r) (we read this for voltage)
        0b001 - Alert Status (r/w) (ignored)
        0b010 - Configuration (r/w) (we configure it for auto-conversion mode)
        0b011 - Low Limit (r/w) (We leave this as 0)
        0b100 - High Limit (r/w) (We leave this as 4096 or 2**12)
        0b110 - Lowest Conversion to Date (r/w) (ignored)
        0b111 - Highest Conversion to Date (r/w) (ignored)
        """

        for key, is_configured in self.configured_adcs.items():

            if is_configured: # Assume ADC is configured. Read conversion result
                try:

                    conversion_result_register = 0b00000000
                    conversion_result_length_in_bytes = 2
                    
                    read = self.bus.read_i2c_block_data(ADC_ADDRESSES[key], conversion_result_register, conversion_result_length_in_bytes)

                    first_byte_data = read[0] << 6 # This will contain the 4 MSBs
                    second_byte_data = read[1] >> 2  # This will contain the 6 LSBs
                    
                    # Combine the first and second byte conversion result data to get the 10 bit value,
                    # Divide this by 3.3/(2**10) since analog input percision is 1024 from 0 to 3.3V
                    adc_read_voltage = (first_byte_data + second_byte_data) * (3.3/(2**10)) 

                    voltage = ((sum(ADC_VOLTAGE_DIVIDER_VALUES[key]))/ADC_VOLTAGE_DIVIDER_VALUES[key][1]) * adc_read_voltage
                    
                except OSError:

                    voltage = 0
                    self.configured_adcs[key] == False

                
                setattr(diagnostics_data, key, voltage) # The read value is converted to 12 bits

            else:

                configuration_address = 0b00000010
                configuration_value = 0b10100000 # (1.7 kilo samples per second)

                try:

                    self.bus.write_byte_data(ADC_ADDRESSES[key], configuration_address, configuration_value)
                    configuration = self.bus.read_byte_data(ADC_ADDRESSES[key],configuration_address)

                    if configuration == configuration_value:
                        self.get_logger().info(f"CONFIGURED ADC {key}")
                        self.configured_adcs[key] = True
                    else:
                        raise OSError
                    
                except OSError:

                    self.configured_adcs[key] = False

        return diagnostics_data
                

    def obtain_temp_sensor_data(self, diagnostics_data):
        '''
        The temperature sensor does not need to be configured, as the default configuration 
        options are good. The registers are as follows:

        0b00 - Temperature Reading (r) (11 MSBs of two bytes)
        0b01 - Configuration (r/w) (ignored)
        0b10 - Hysteresis (r/w) (ignored, used for alerts) (default 75 celcius)
        0b11 - Overtemperature Shutdown Value (r/w) (ignored) (default 80 celcius)

        The alert is indicated by a seperate pin (not an i2c register)
        '''

        temperature_reading_register = 0b00000000
        temperature_reading_register_length_in_bytes = 2 

        for key, address in TEMPERATURE_SENSOR_ADDRESSES.items():
            try:
                read = self.bus.read_i2c_block_data(address, temperature_reading_register, temperature_reading_register_length_in_bytes)

                # Convert to 11 bits
                read = ((read[0] << 8) + read[1]) >> 5 

                # According to the datasheet (https://www.nxp.com/docs/en/data-sheet/LM75B.pdf), the reading is in two's complement form. 
                # The line below obtains magnitude and assigns correct sign

                read = -((0b11111111111 - read) + 1) if read >= 0b10000000000 else read

            except OSError:

                read = 0

            # Data sheet says to use 0.125 for conversion to celcius
            setattr(diagnostics_data, key, float(read * 0.125))

        return diagnostics_data

    def pilot_listener_callback(self, msg): 
        
        if not self.autonomous_mode_active or (msg.is_autonomous and self.autonomous_mode_active):
            thruster_values = self.rov_math(msg)

            if self.bus is not None:
                
                for thruster_position in THRUSTER_CHANNELS:
                    self.connected_channels[THRUSTER_CHANNELS[thruster_position]].fly(thruster_values[thruster_position])

                    # Thrusters should be armed by the time the first pilot input is recieved
                    if not self.connected_channels[THRUSTER_CHANNELS[thruster_position]].thruster_armed:
                        self.get_logger().error(f"Thruster {thruster_position} not armed")
                
                self.tick_thrusters()
                self.stm32_communications(msg)
        
        if msg.enter_auto_mode:
            if not self.autonomous_mode_active:
                self.autonomous_mode_active = True
                self.send_autonomous_mode_goal()
            else:
                future = self.goal_handle.cancel_goal_async()
                future.add_done_callback(self.cancel_done)

    def stm32_communications(self, controller_inputs):
        '''
        Communications to the STM32 is required for the 12 DC motor (claw linear actuator),
        stepper motor (claw rotation), dimmable LEDs, and outside temperature probe readings.

        The eer properity communication protocol is (currently) based only on writes. Each action will be achieved through:

        self.bus.write_byte_data(STM32_ADDRESS, action_register, value)

        Possible actions and associated registers are as follows:

        STM32 Address: 0x69

        DC Motor Structure (Bytes listed from left to right): 
        Byte 1: Command and Motor Selection
        -- Command: 0 (Hex)
        -- Motor Number: 1 or 2 (Hex)
        -- Example Byte: 0x01 (DC Motor 1)
        Byte 2: Motor Direction and Speed (UNUSED)
        -- Motor Direction: 0 or 1 (will figure out which is forward when we test)
        -- Speed: Unused right now so set to 0
        -- Example Byte: 0x10 (Direction = 1)

        STEPPER Structure:
        Byte 1: Command and Stepper Selection
        -- Command: 1 (hex)
        -- Stepper Selection: 1 or 2
        -- Example: 0x12 (Stepper 2)
        Byte 2: Direction and Speed
        -- Direction: 0 or 1
        -- Speed: Unused right now
        -- Example: 0x10 (Go in direction 1)

        LED Structure:
        Byte 1: Command and LED Selection
        -- Command: 2 (Hex)
        -- LED Selection: 1-4
        -- Example: 0x24 (LED 4)
        Byte 2: Brightness (0-99)

        Outside Temperature Probe:
        Byte 1: Select probe
        -- Command: 3 (hex)
        -- Temperature Probe Selection: 1
        -- Example: 0x31
        Byte 2-3: Data
        --Format: 0bSSSS S(2^6)(2^5)(2^4) (2^3)(2^2)(2^1)(2^0) (2^-1) (2^-2) (2^-3) (2^-4)
        -- S is for signed digits, all 0 or 1 depending on sign
        '''
        
        if self.bus is not None:

            if controller_inputs.open_claw or controller_inputs.close_claw:
                claw_value = 0x10 if controller_inputs.open_claw else 0x00
                try:
                    self.bus.write_byte_data(STM32_ADDRESS, 0x01, claw_value)
                except OSError:
                    self.get_logger().error(f"COULD NOT PERFORM ACTION WITH ACTION ADDRESS {(0x10,claw_value)}")
            else:
                try:
                    self.bus.write_byte_data(STM32_ADDRESS, 0x01, 0xaa) # Send stop signal
                except OSError:
                    pass

            if controller_inputs.turn_stepper_cw or controller_inputs.turn_stepper_ccw:
                stepper_value = 0x10 if controller_inputs.turn_stepper_cw else 0x00
                try:
                    self.bus.write_byte_data(STM32_ADDRESS, 0x02, stepper_value)
                except OSError:
                    self.get_logger().error(f"COULD NOT PERFORM ACTION WITH ACTION ADDRESS {(0x02,stepper_value)}")
            else:
                try:
                    self.bus.write_byte_data(STM32_ADDRESS, 0x02, 0xaa) # Send stop signal
                except OSError:
                    pass
                        

            led_addresses = (0x21,0x22,0x23,0x24)

            if controller_inputs.brighten_led or controller_inputs.dim_led:
                self.headlight_led_brightness += 10 if controller_inputs.brighten_led else -10

                if self.headlight_led_brightness > 99:
                    self.headlight_led_brightness = 99
                elif self.headlight_led_brightness < 0:
                    self.headlight_led_brightness = 0

                for led_address in led_addresses:
                    try:
                        self.bus.write_byte_data(STM32_ADDRESS, led_address, self.headlight_led_brightness)
                    except OSError:
                        self.get_logger().error(f"COULD NOT PERFORM ACTION WITH ACTION ADDRESS {(led_address,self.headlight_led_brightness)}")
            
            outside_temperature_probe_register = 0x30
            
            if controller_inputs.read_outside_temperature_probe:

                diagnostics_data = DiagnosticsData()

                diagnostics_data = self.obtain_adc_data(diagnostics_data) 
                diagnostics_data = self.obtain_temp_sensor_data(diagnostics_data)

                if self.imu_detected:
                    diagnostics_data = self.obtain_imu_data(diagnostics_data)

                self.diagnostics_data_publisher.publish(diagnostics_data)
                    
                outside_temp_probe_data = OutsideTempProbeData()
                
                stm_locked = False

                try:
                    self.bus.write_byte_data(STM32_ADDRESS, outside_temperature_probe_register, 1)
                    stm_locked = True
                except OSError:
                    self.get_logger().error("Cannot write to STM32 for temperature probe")

                # Data will arrive based on this format (https://cdn.sparkfun.com/datasheets/Sensors/Temp/DS18B20.pdf)
                
                readLSBs = -1
                readMSBs = -1
                while stm_locked:
                    try:
                        if readLSBs == -1:
                            readLSBs = self.bus.read_byte(STM32_ADDRESS)
                        elif readMSBs == -1:
                            readMSBs = self.bus.read_byte(STM32_ADDRESS)
                        else:
                            stm_locked = False
                    except:
                        continue

                outside_temp_probe_data.temperature = float((readMSBs<<8)|readLSBs)/16

                self.outside_temperature_probe_data_publisher.publish(outside_temp_probe_data)

                
                


    def rov_math(self, controller_inputs):
        '''
        Determines how much power to give to each thruster based on pilot input.
        '''

        thruster_values = {}

        ###########################################
        ############ OLD THRUSTER MATH ############
        ###########################################

        # surge = controller_inputs.surge * self.power_multiplier * self.surge_multiplier * 0.01
        # sway = controller_inputs.sway * self.power_multiplier * self.sway_multiplier * 0.01
        # yaw = controller_inputs.yaw * self.power_multiplier * self.yaw_multiplier * 0.01

        # if controller_inputs.heave_up or controller_inputs.heave_down:
        #     heave = ((self.power_multiplier * self.heave_multiplier) if controller_inputs.heave_up else 0) + ((-self.power_multiplier * self.heave_multiplier) if controller_inputs.heave_down else 0)
        # else:
        #     heave = controller_inputs.heave * self.power_multiplier * self.heave_multiplier * 0.01  

        # if controller_inputs.pitch_up or controller_inputs.pitch_down:
        #     pitch = ((self.power_multiplier * self.pitch_multiplier) if controller_inputs.pitch_up else 0) + ((-self.power_multiplier * self.pitch_multiplier) if controller_inputs.pitch_down else 0)
        # else:
        #     pitch = controller_inputs.pitch * self.power_multiplier * self.pitch_multiplier * 0.01  

        # sum_of_magnitudes_of_linear_movements = abs(surge) + abs(sway) + abs(heave)
        # sum_of_magnitudes_of_rotational_movements = abs(pitch) + abs(yaw)

        # strafe_power = sqrt(surge**2 + sway**2 + heave**2)
        # strafe_scaling_coefficient = strafe_power / (sum_of_magnitudes_of_linear_movements) if strafe_power else 0
        # strafe_average_coefficient = strafe_power / (strafe_power + sum_of_magnitudes_of_rotational_movements) if strafe_power or sum_of_magnitudes_of_rotational_movements else 0  
        # combined_strafe_coefficient = strafe_scaling_coefficient * strafe_average_coefficient
        # rotation_average_coefficient = sum_of_magnitudes_of_rotational_movements / (strafe_power + sum_of_magnitudes_of_rotational_movements) if strafe_power or sum_of_magnitudes_of_rotational_movements else 0

        # # The to decimal adjustment factor is 1.85 (max value that each thruster value can be)

        # # Calculations below are based on thruster positions
        # thruster_values["for-port-bot"] = (((-surge)+(sway)+(heave)) * combined_strafe_coefficient + ((pitch)+(yaw)) * rotation_average_coefficient) / 1.85
        # thruster_values["for-star-bot"] = (((-surge)+(-sway)+(heave)) * combined_strafe_coefficient + ((pitch)+(-yaw)) * rotation_average_coefficient) / 1.85
        # thruster_values["aft-port-bot"] = (((surge)+(sway)+(heave)) * combined_strafe_coefficient + ((-pitch)+(-yaw)) * rotation_average_coefficient) / -1.85
        # thruster_values["aft-star-bot"] = (((surge)+(-sway)+(heave)) * combined_strafe_coefficient + ((-pitch)+(yaw)) * rotation_average_coefficient) / 1.85
        # thruster_values["for-port-top"] = (((-surge)+(sway)+(-heave)) * combined_strafe_coefficient + ((-pitch)+(yaw)) * rotation_average_coefficient) / -1.85
        # thruster_values["for-star-top"] = (((-surge)+(-sway)+(-heave)) * combined_strafe_coefficient + ((-pitch)+(-yaw)) * rotation_average_coefficient) / -1.85
        # thruster_values["aft-port-top"] = (((surge)+(sway)+(-heave)) * combined_strafe_coefficient + ((pitch)+(-yaw)) * rotation_average_coefficient) / -1.85
        # thruster_values["aft-star-top"] = (((surge)+(-sway)+(-heave)) * combined_strafe_coefficient + ((pitch)+(yaw)) * rotation_average_coefficient) / -1.85

        ###########################################
        ############ NEW THRUSTER MATH ############
        ###########################################

        # Power adjustment factor of 0.65 added to avoid brownout

        surge = controller_inputs.surge * self.power_multiplier * self.surge_multiplier * 0.01 * 0.65
        sway = controller_inputs.sway * self.power_multiplier * self.sway_multiplier * 0.01 * 0.65
        yaw = controller_inputs.yaw * self.power_multiplier * self.yaw_multiplier * 0.01 * 0.65

        if controller_inputs.heave_up or controller_inputs.heave_down:
            controller_inputs.heave = (100 if controller_inputs.heave_up else 0) + (-100 if controller_inputs.heave_down else 0) 
            
        heave = controller_inputs.heave * self.power_multiplier * self.heave_multiplier * 0.01  * 0.65

        if controller_inputs.pitch_up or controller_inputs.pitch_down:
            controller_inputs.pitch = (100 if controller_inputs.pitch_up else 0) + (-100 if controller_inputs.pitch_down else 0)
        
        pitch = controller_inputs.pitch * self.power_multiplier * self.pitch_multiplier * 0.01  * 0.65

        sum_of_magnitudes_of_pilot_input = abs(surge) + abs(sway) + abs(heave) + abs(pitch) + abs(yaw)

        # These adjustment factors determine how much to decrease power in each thruster due to multipliers.
        # The first mutliplication term determines the total % to remove of the inital thruster direction,
        # The second term scales that thruster direction down based on what it makes up of the sum of pilot input, and also applies a sign

        surge_adjustment = (1 - (self.power_multiplier * 0.65 * self.surge_multiplier * abs(controller_inputs.surge) * 0.01)) * (surge/sum_of_magnitudes_of_pilot_input if sum_of_magnitudes_of_pilot_input else 0)  
        sway_adjustment = (1 - (self.power_multiplier * 0.65 * self.sway_multiplier * abs(controller_inputs.sway) * 0.01)) * (sway/sum_of_magnitudes_of_pilot_input if sum_of_magnitudes_of_pilot_input else 0) 
        heave_adjustment = (1 - (self.power_multiplier * 0.65 * self.heave_multiplier * abs(controller_inputs.heave) * 0.01)) * (heave/sum_of_magnitudes_of_pilot_input if sum_of_magnitudes_of_pilot_input else 0) 
        pitch_adjustment = (1 - (self.power_multiplier * 0.65 * self.pitch_multiplier * abs(controller_inputs.pitch) * 0.01)) * (pitch/sum_of_magnitudes_of_pilot_input if sum_of_magnitudes_of_pilot_input else 0) 
        yaw_adjustment = (1 - (self.power_multiplier * 0.65 * self.yaw_multiplier * abs(controller_inputs.yaw) * 0.01)) * (yaw/sum_of_magnitudes_of_pilot_input if sum_of_magnitudes_of_pilot_input else 0) 

        # Ensure to scale the thruster values down such that they don't exceed 1 in magnitude
        thruster_scaling_coefficient = 1 / sum_of_magnitudes_of_pilot_input if sum_of_magnitudes_of_pilot_input else 0

        # Calculations below are based on thruster positions:

        # First term:
        # The net pilot input based on how it applies to the specifc thruster (some are reveresed) is calculated.
        # Then, this is scaled down by the thruster scaling coefficent such that the max absolute value it can attain is 1.
        # This will properly activate distribute load and direction among thrusters such that the desired movement is reached,
        # but the first term cancels out the effect of the thruster multipliers 

        # Directional adjustment factors:
        # These adjustment factors will never increase the power going to a single thruster.
        # They will only serve to proportionally decrease it in order to reduce power in a certain direction. 
         
        thruster_values["for-port-bot"] = ((((-surge)+(sway)+(heave)+(pitch)+(yaw)) * thruster_scaling_coefficient) + surge_adjustment - sway_adjustment - heave_adjustment - pitch_adjustment - yaw_adjustment)
        thruster_values["for-star-bot"] = ((((-surge)+(-sway)+(heave)+(pitch)+(-yaw)) * thruster_scaling_coefficient) + surge_adjustment + sway_adjustment - heave_adjustment - pitch_adjustment + yaw_adjustment)
        thruster_values["aft-port-bot"] = -1 * ((((surge)+(sway)+(heave)+ (-pitch)+(-yaw)) * thruster_scaling_coefficient) - surge_adjustment - sway_adjustment - heave_adjustment + pitch_adjustment + yaw_adjustment)
        thruster_values["aft-star-bot"] = ((((surge)+(-sway)+(heave)+(-pitch)+(yaw)) * thruster_scaling_coefficient) - surge_adjustment + sway_adjustment - heave_adjustment + pitch_adjustment - yaw_adjustment)
        thruster_values["for-port-top"] = -1 * ((((-surge)+(sway)+(-heave)+(-pitch)+(yaw)) * thruster_scaling_coefficient) + surge_adjustment - sway_adjustment + heave_adjustment + pitch_adjustment - yaw_adjustment)
        thruster_values["for-star-top"] = -1 * ((((-surge)+(-sway)+(-heave)+(-pitch)+(-yaw)) * thruster_scaling_coefficient) + surge_adjustment + sway_adjustment + heave_adjustment + pitch_adjustment + yaw_adjustment)
        thruster_values["aft-port-top"] = -1 * ((((surge)+(sway)+(-heave)+(pitch)+(-yaw)) * thruster_scaling_coefficient) - surge_adjustment - sway_adjustment + heave_adjustment - pitch_adjustment + yaw_adjustment)
        thruster_values["aft-star-top"] = -1 * ((((surge)+(-sway)+(-heave)+(pitch)+(yaw)) * thruster_scaling_coefficient) - surge_adjustment + sway_adjustment + heave_adjustment - pitch_adjustment - yaw_adjustment)

        ####################################################################
        ############################## DEBUG ###############################
        ####################################################################

        # Calculations below will calculate and display the net movement in all directions based on vector analysis
        # Ensure to also uncomment the debugger attribute in the init method

        # if self.debugger_mode:

        #     from math import cos, pi
            
        #     raw_inputs = String()
        #     raw_inputs.data = f"""
        #     # Surge: {surge}\n
        #     # Sway: {sway}\n
        #     # Heave: {heave}\n
        #     # Pitch: {pitch}\n
        #     # Yaw: {yaw}"""
        #     self.debugger.publish(raw_inputs)

        #     thruster_values_debug = String()
        #     thruster_values_debug.data = f"""
        #     forportbot:{thruster_values["for-port-bot"]},for star bot:{thruster_values["for-star-bot"]},aftportbot:{thruster_values["aft-port-bot"]},aftstarbot:{thruster_values["aft-star-bot"]},forporttop:{thruster_values["for-port-top"]},
        #     forstartop:{thruster_values["for-star-top"]},aft port top:{thruster_values["aft-port-top"]},aftstartop:{thruster_values["aft-star-top"]}"""
        #     self.debugger.publish(thruster_values_debug)

        #     net_surge = cos(pi/3)*((-thruster_values["for-port-bot"]) + (-thruster_values["for-star-bot"]) + (-thruster_values["aft-port-bot"]) + (thruster_values["aft-star-bot"]) + (thruster_values["for-port-top"]) + (thruster_values["for-star-top"]) + (-thruster_values["aft-port-top"]) + (-thruster_values["aft-star-top"]))
        #     net_sway = cos(pi/3)*((thruster_values["for-port-bot"]) + (-thruster_values["for-star-bot"]) + (-thruster_values["aft-port-bot"]) + (-thruster_values["aft-star-bot"]) + (-thruster_values["for-port-top"]) + (thruster_values["for-star-top"]) + (-thruster_values["aft-port-top"]) + (thruster_values["aft-star-top"]))
        #     net_heave = cos(pi/3)*((thruster_values["for-port-bot"]) + (thruster_values["for-star-bot"]) + (-thruster_values["aft-port-bot"]) + (thruster_values["aft-star-bot"]) + (thruster_values["for-port-top"]) + (thruster_values["for-star-top"]) + (thruster_values["aft-port-top"]) + (thruster_values["aft-star-top"]))
            
        #     net_pitch = cos(pi/4)*((thruster_values["for-port-bot"]) + (thruster_values["for-star-bot"]) + (thruster_values["aft-port-bot"]) + (-thruster_values["aft-star-bot"]) + (thruster_values["for-port-top"]) + (thruster_values["for-star-top"]) + (-thruster_values["aft-port-top"]) + (-thruster_values["aft-star-top"]))
        #     net_yaw = cos(pi/4)*((thruster_values["for-port-bot"]) + (-thruster_values["for-star-bot"]) + (thruster_values["aft-port-bot"]) + (thruster_values["aft-star-bot"]) + (-thruster_values["for-port-top"]) + (thruster_values["for-star-top"]) + (thruster_values["aft-port-top"]) + (-thruster_values["aft-star-top"]))

        #     net_movement_vectors = String()
        #     net_movement_vectors.data = f"""Net Surge:{net_surge},Net Sway:{net_sway},Net Heave:{net_heave},Net Pitch:{net_pitch},Net Yaw:{net_yaw}"""
        #     self.debugger.publish(net_movement_vectors)

        ###########################################################################################
        ############################## INIDIVISUAL THRUSTER TESITNG ###############################
        ###########################################################################################

        if self.thruster_calibration_mode:
            
            current_thruster_and_value = String()

            if controller_inputs.brighten_led:
                if self.current_thruster == 0:
                    self.current_thruster = 7
                else:
                    self.current_thruster -= 1

                current_thruster_and_value.data = f"{self.current_thruster}: {self.surge}"
                self.debugger.publish(current_thruster_and_value)

            elif controller_inputs.dim_led:
                if self.current_thruster == 7:
                    self.current_thruster = 0
                else:
                    self.current_thruster += 1

                current_thruster_and_value.data = f"{self.current_thruster}: {self.surge}"
                self.debugger.publish(current_thruster_and_value)

            for thruster_position, channel in THRUSTER_CHANNELS.items():
                if channel == self.current_thruster:
                    thruster_values[thruster_position] = surge
                else:   
                    thruster_values[thruster_position] = 0

        
        ###########################################################################################
        ###########################################################################################
        ###########################################################################################
        return thruster_values
    

    def send_autonomous_mode_goal(self):

        self.fetch_brain_coral_hsv_colour_bounds()
        goal_msg = AutoMode.Goal()
        goal_msg.is_for_sim = False

        # HSV (hue,shade,value) bounds for filtering brain coral area
        goal_msg.lower_hsv_bound = self.brain_coral_hsv_colour_bounds["lower_hsv"]
        goal_msg.upper_hsv_bound = self.brain_coral_hsv_colour_bounds["upper_hsv"]

        goal_msg.starting_power = int(self.power_multiplier * 100)
        goal_msg.starting_surge = int(self.surge_multiplier * 100)
        goal_msg.starting_sway = int(self.sway_multiplier * 100)
        goal_msg.starting_heave = int(self.heave_multiplier * 100)
        goal_msg.starting_pitch = int(self.pitch_multiplier * 100)
        goal_msg.starting_yaw = int(self.yaw_multiplier * 100) 

        self._action_client.wait_for_server()

        self._send_goal_future = self._action_client.send_goal_async(goal_msg, feedback_callback=self.feedback_callback)

        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def fetch_brain_coral_hsv_colour_bounds(self):

        hsv_colour_bounds_request = HSVColours.Request()

        # load_to_database = False indicates loading FROM database
        hsv_colour_bounds_request.load_to_database = False 

        # Ensure the database server is up before continuing 
        self.brain_coral_hsv_colour_bounds_client.wait_for_service()
            
        future = self.brain_coral_hsv_colour_bounds_client.call(hsv_colour_bounds_request)
        
        if future.success: # Means that HSV colours were stored in the database at this time
            self.brain_coral_hsv_colour_bounds["upper_hsv"] = future.upper_hsv
            self.brain_coral_hsv_colour_bounds["lower_hsv"] = future.lower_hsv
        else:
            self.get_logger().info("No HSV colour bounds stored in database. Will keep using default.")    

    def goal_response_callback(self, future):
        self.goal_handle = future.result()
        if not self.goal_handle.accepted:
            self.get_logger().info('Goal rejected :(')
            return

        self.get_logger().info('Goal accepted :)')

        self._get_result_future = self.goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        result = future.result().result
        autonomous_mode_status = String()
        autonomous_mode_status.data = "Autonomous Mode off, {0}".format("Mission Success" if result.success else "Mission Failed")
        self.autonomous_mode_publisher.publish(autonomous_mode_status)
        self.autonomous_mode_active = False

    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        autonomous_mode_status = String()
        autonomous_mode_status.data = "Autonomous Mode on, {0}".format(feedback.status)
        self.autonomous_mode_publisher.publish(autonomous_mode_status)
    
    def cancel_done(self, future):
        cancel_response = future.result()
        if len(cancel_response.goals_canceling) > 0:
            autonomous_mode_status = String()
            autonomous_mode_status.data = 'Auto mode successfully canceled'
            self.autonomous_mode_publisher.publish(autonomous_mode_status)
        else:
            autonomous_mode_status = String()
            autonomous_mode_status.data = 'Auto mode failed to cancel'
            self.autonomous_mode_publisher.publish(autonomous_mode_status)

def main(args=None):
    rclpy.init(args=args)

    i2c_master = I2CMaster()

    # We use a MultiThreadedExecutor to handle incoming goal requests concurrently
    executor = MultiThreadedExecutor()
    rclpy.spin(i2c_master, executor=executor)

    rclpy.spin(i2c_master)

    i2c_master.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()