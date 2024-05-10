import rclpy
from rclpy.node import Node, MutuallyExclusiveCallbackGroup

from eer_messages.msg import ThrusterMultipliers, PilotInput, IMUData, ADCData, TempSensorData, OutsideTempProbeData

import board
from smbus2 import SMBus
from adafruit_bno055 import BNO055_I2C

from math import sqrt


# Thurster channels are based on Beaumont
THRUSTER_CHANNELS = {
    "for-port-top": 1, 
    "for-star-top": 0, 
    "aft-port-top": 4,
    "aft-star-top": 7,
    "for-port-bot": 2,
    "for-star-bot": 6,
    "aft-port-bot": 5,
    "aft-star-bot": 3
}

# In the thruster class, the target speed is set by the user. Each thruster accelerates towards the target speed by the acceleration below
THRUSTER_ACCELERATION = 1 

# Determines how often each thruster accelerates towards the target speed set by the user
THRUSTER_TICK_PERIOD = 0.01

RP2040_ADDRESS = 0x08

STM32_ADDRESS = 0x69 

ADC_ADDRESSES = {
    "adc_48v_bus": 0x55,
    "adc_12v_bus":0x56,
    "adc_5v_bus":0x59
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
IMU_REQUESTS_PERIOD = 1
ADC_REQUESTS_PERIOD = 1
TEMP_SENSOR_REQUESTS_PERIOD = 1

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

        # Callback group for reciving topsides input and communicated to i2c bus
        i2c_master_callback_group = MutuallyExclusiveCallbackGroup()

        # Subscribers
        self.copilot_listener = self.create_subscription(ThrusterMultipliers, 'thruster_multipliers', self.copilot_listener_callback, 100, callback_group=i2c_master_callback_group)
        self.pilot_listener = self.create_subscription(PilotInput, 'controller_input', self.pilot_listener_callback, 1, callback_group=i2c_master_callback_group) # input recieved at 10Hz

        # Publishers
        self.imu_data_publisher = self.create_publisher(IMUData, "imu", 10)
        self.adc_data_publisher = self.create_publisher(ADCData, "adc", 10)
        self.temp_sensor_data_publisher = self.create_publisher(TempSensorData, "board_temp", 10)
        self.outside_temperature_probe_data_publisher = self.create_publisher(OutsideTempProbeData, "outside_temp_probe", 10)

        # Debugger publisher
        # from std_msgs.msg import String
        # self.debugger = self.create_publisher(String, 'debugger', 10)

        # For finiding center speed and testing indivisual thrusters
        # self.current_thruster = 0

        # prevent unused variable warning
        self.copilot_listener 
        self.pilot_listener

        self.headlight_led_brightness = 50
        
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

        if self.i2c is not None:
            # Connect to BNO055
            try:
                self.imu_sensor = BNO055_I2C(self.i2c)
                self.last_temperature_val = 0xFFFF # per recommendation on (https://learn.adafruit.com/adafruit-bno055-absolute-orientation-sensor/python-circuitpython)

                self.imu_timer = self.create_timer(IMU_REQUESTS_PERIOD, self.obtain_imu_data, callback_group=i2c_master_callback_group)

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

        if self.bus is not None:
            self.thruster_timer = self.create_timer(THRUSTER_TICK_PERIOD, self.tick_thrusters, callback_group=i2c_master_callback_group)


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

            self.adc_timer = self.create_timer(ADC_REQUESTS_PERIOD, self.obtain_adc_data, callback_group=i2c_master_callback_group)

        #################################################################
        ###################### Temperature Sensors ######################
        #################################################################

        if self.bus is not None:

            self.temp_sensor_timer = self.create_timer(TEMP_SENSOR_REQUESTS_PERIOD, self.obtain_temp_sensor_data, callback_group=i2c_master_callback_group)
        

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
            # self.get_logger().info(str(self.connected_channels[channel].current)) # Temporary, useful for debug

    def obtain_imu_data(self):
        '''
        Grabs relevant information form the IMU on the I2C Bus.
        Every time a piece of data is accessed, that is an i2c read transation. These transactions finish by the time the value is
        returned. See (https://github.com/adafruit/Adafruit_CircuitPython_BNO055/blob/main/adafruit_bno055.py).
        '''
        imu_data = IMUData()

        imu_data.temperature = self.imu_sensor.temperature

        # This code below was recommended by Adafruit (https://learn.adafruit.com/adafruit-bno055-absolute-orientation-sensor/python-circuitpython)
        if abs(imu_data.temperature - self.last_temperature_val) == 128:
            imu_data.temperature = 0b00111111 & imu_data.temperature
        self.last_temperature_val = imu_data.temperature

        # Below assignments only work because the imu returns size 3 tuples
        # The ROS IMUData message type demands size 3 arrays
        imu_data.acceleration = list(self.imu_sensor.acceleration)
        imu_data.magnetic = list(self.imu_sensor.magnetic) 
        imu_data.euler = list(self.imu_sensor.euler)
        imu_data.linear_acceleration = list(self.imu_sensor.linear_acceleration)

        self.imu_data_publisher.publish(imu_data)

        # imu_debug_data = String()
        # imu_debug_data.data = str(imu_data)
        # self.debugger.publish(imu_debug_data)

    def obtain_adc_data(self):
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

        adc_data = ADCData()

        for key, is_configured in self.configured_adcs.items():

            if is_configured: # Assume ADC is configured. Read conversion result
                try:

                    conversion_result_register = 0b00000000
                    conversion_result_length_in_bytes = 2
                    
                    read = self.bus.read_i2c_block_data(ADC_ADDRESSES[key], conversion_result_register, conversion_result_length_in_bytes)

                    read_12_bit = int("0b" + bin(read[0])[6:] + bin(read[1])[2:8],2)
                    
                except OSError:

                    read_12_bit = 0
                    self.configured_adcs[key] == False

                
                setattr(adc_data, key, read_12_bit) # The read value is converted to 12 bits

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
                

        self.adc_data_publisher.publish(adc_data)

        # adc_debug_data = String()
        # adc_debug_data.data = str(adc_data)
        # self.debugger.publish(adc_debug_data)

    def obtain_temp_sensor_data(self):
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

        temp_sensor_data = TempSensorData()

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
            setattr(temp_sensor_data, key, float(read * 0.125))

        self.temp_sensor_data_publisher.publish(temp_sensor_data) 
        
        # temp_sensor_debug_data = String()
        # temp_sensor_debug_data.data = str(temp_sensor_data)
        # self.debugger.publish(temp_sensor_debug_data)

    def pilot_listener_callback(self, msg): 

        thruster_values = self.rov_math(msg)

        if self.bus is not None:
            
            for thruster_position in THRUSTER_CHANNELS:
                self.connected_channels[THRUSTER_CHANNELS[thruster_position]].fly(thruster_values[thruster_position])

                # Thrusters should be armed by the time the first pilot input is recieved
                if not self.connected_channels[THRUSTER_CHANNELS[thruster_position]].thruster_armed:
                    self.get_logger().error(f"Thruster {thruster_position} not armed")

            self.stm32_communications(msg)

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
                        self.get_logger().info(f"SENDING LED")
                    except OSError:
                        self.get_logger().error(f"COULD NOT PERFORM ACTION WITH ACTION ADDRESS {(led_address,self.headlight_led_brightness)}")
            
            # outside_temperature_probe_register = 0x17
            # temperature_probe_register_length_in_bytes = 2
            
            # if controller_inputs.read_outside_temperature_probe:
                    
            #     outside_temp_probe_data = OutsideTempProbeData()

            #     # Data will arrive based on this format (https://cdn.sparkfun.com/datasheets/Sensors/Temp/DS18B20.pdf)

            #     read = self.bus.read_i2c_block_data(STM32_ADDRESS, outside_temperature_probe_register, temperature_probe_register_length_in_bytes)

            #     # Convert from 12 bit two's complement binary to string
            #     read_12_bit_twos_complement_str = '{0:016b}'.format((read[0] << 8) + read[1])[4:]

            #     # Convert from 12 bit two's complement string to signed binary

            #     read_12_bit_twos_complement = int(f"0b{read_12_bit_twos_complement_str}",2)

            #     read_12_bit_magnitude_str = '{0:016b}'.format(((0b111111111111 - read_12_bit_twos_complement) + 1) if read_12_bit_twos_complement >= 0b100000000000 else read_12_bit_twos_complement)[4:] 

            #     # Convert to float32

            #     outside_temp_probe_data.data = (-1 if int(read_12_bit_twos_complement_str[0]) else 1) * (int(f"0b{read_12_bit_magnitude_str[:8]}",2) + 0.5 * int(read_12_bit_magnitude_str[8]) + 0.25 * int(read_12_bit_magnitude_str[9]) + 0.125 * int(read_12_bit_magnitude_str[10]) + 0.0625 * int(read_12_bit_magnitude_str[11]))

            #     if outside_temp_probe_data.data == 0xF0:
            #         self.get_logger().error(f"COULD NOT PERFORM ACTION WITH ACTION ADDRESS {bin(outside_temperature_probe_register)}")
            #     elif outside_temp_probe_data.data == 0x00:
            #         self.get_logger().error(f"COULD NOT COMMUNICATE TO STM32")

            #     self.outside_temperature_probe_data_publisher.publish(outside_temp_probe_data)

                
                


    def rov_math(self, controller_inputs):
        '''
        Determines what power to give to each thruster based on pilot input.

        # TODO Current issue (which was an issue with previous years aswell) is that the absolute values for each thruster can exceed 1 up to a max value of sqrt(2). Can be fixed by adjusting thruster speeds or approperiate scaling.
        '''

        thruster_values = {}

        surge = controller_inputs.surge * self.power_multiplier * self.surge_multiplier * 0.01
        sway = controller_inputs.sway * self.power_multiplier * self.sway_multiplier * 0.01
        yaw = controller_inputs.yaw * self.power_multiplier * self.yaw_multiplier * 0.01

        if controller_inputs.heave_up or controller_inputs.heave_down:
            heave = ((self.power_multiplier * self.heave_multiplier) if controller_inputs.heave_up else 0) + ((-self.power_multiplier * self.heave_multiplier) if controller_inputs.heave_down else 0)
        else:
            heave = controller_inputs.heave * self.power_multiplier * self.heave_multiplier * 0.01  

        if controller_inputs.pitch_up or controller_inputs.pitch_down:
            pitch = ((self.power_multiplier * self.pitch_multiplier) if controller_inputs.pitch_up else 0) + ((-self.power_multiplier * self.pitch_multiplier) if controller_inputs.pitch_down else 0)
        else:
            pitch = controller_inputs.pitch * self.power_multiplier * self.pitch_multiplier * 0.01  

        sum_of_magnitudes_of_linear_movements = abs(surge) + abs(sway) + abs(heave)
        sum_of_magnitudes_of_rotational_movements = abs(pitch) + abs(yaw)

        strafe_power = sqrt(surge**2 + sway**2 + heave**2)
        strafe_scaling_coefficient = strafe_power / (sum_of_magnitudes_of_linear_movements) if strafe_power else 0
        strafe_average_coefficient = strafe_power / (strafe_power + sum_of_magnitudes_of_rotational_movements) if strafe_power or sum_of_magnitudes_of_rotational_movements else 0  
        combined_strafe_coefficient = strafe_scaling_coefficient * strafe_average_coefficient
        rotation_average_coefficient = sum_of_magnitudes_of_rotational_movements / (strafe_power + sum_of_magnitudes_of_rotational_movements) if strafe_power or sum_of_magnitudes_of_rotational_movements else 0

        # The to decimal adjustment factor is 1.85 (max value that each thruster value can be)

        # Calculations below are based on thruster positions
        thruster_values["for-port-bot"] = (((-surge)+(sway)+(heave)) * combined_strafe_coefficient + ((pitch)+(yaw)) * rotation_average_coefficient) / 1.85
        thruster_values["for-star-bot"] = (((-surge)+(-sway)+(heave)) * combined_strafe_coefficient + ((pitch)+(-yaw)) * rotation_average_coefficient) / 1.85
        thruster_values["aft-port-bot"] = (((surge)+(sway)+(heave)) * combined_strafe_coefficient + ((-pitch)+(-yaw)) * rotation_average_coefficient) / -1.85
        thruster_values["aft-star-bot"] = (((surge)+(-sway)+(heave)) * combined_strafe_coefficient + ((-pitch)+(yaw)) * rotation_average_coefficient) / 1.85
        thruster_values["for-port-top"] = (((-surge)+(sway)+(-heave)) * combined_strafe_coefficient + ((-pitch)+(yaw)) * rotation_average_coefficient) / -1.85
        thruster_values["for-star-top"] = (((-surge)+(-sway)+(-heave)) * combined_strafe_coefficient + ((-pitch)+(-yaw)) * rotation_average_coefficient) / -1.85
        thruster_values["aft-port-top"] = (((surge)+(sway)+(-heave)) * combined_strafe_coefficient + ((pitch)+(-yaw)) * rotation_average_coefficient) / -1.85
        thruster_values["aft-star-top"] = (((surge)+(-sway)+(-heave)) * combined_strafe_coefficient + ((pitch)+(yaw)) * rotation_average_coefficient) / -1.85

        ####################################################################
        ############################## DEBUG ###############################
        ####################################################################

        # Calculations below will calculate and display the net movement in all directions based on vector analysis
        # Ensure to also uncomment the debugger attribute in the init method

        # from math import cos, pi
        
        # raw_inputs = String()
        # raw_inputs.data = f"""
        # # Surge: {surge}\n
        # # Sway: {sway}\n
        # # Heave: {heave}\n
        # # Pitch: {pitch}\n
        # # Yaw: {yaw}"""
        # self.debugger.publish(raw_inputs)

        # coefficients = String()
        # coefficients.data = f"""Srfe Pow: {strafe_power}, Strfe scal: {strafe_scaling_coefficient}, Strfe avg: {strafe_average_coefficient}, rot avg: {rotation_average_coefficient}"""
        # self.debugger.publish(coefficients)

        # from std_msgs.msg import String

        # thruster_values_debug = String()
        # thruster_values_debug.data = f"""
        # forportbot:{thruster_values["for-port-bot"]},for star bot:{thruster_values["for-star-bot"]},aftportbot:{thruster_values["aft-port-bot"]},aftstarbot:{thruster_values["aft-star-bot"]},forporttop:{thruster_values["for-port-top"]},
        # forstartop:{thruster_values["for-star-top"]},aft port top:{thruster_values["aft-port-top"]},aftstartop:{thruster_values["aft-star-top"]}"""
        # self.debugger.publish(thruster_values_debug)

        # net_surge = cos(pi/3)*((-thruster_values["for-port-bot"]) + (-thruster_values["for-star-bot"]) + (thruster_values["aft-port-bot"]) + (thruster_values["aft-star-bot"]) + (-thruster_values["for-port-top"]) + (-thruster_values["for-star-top"]) + (thruster_values["aft-port-top"]) + (thruster_values["aft-star-top"]))
        # net_sway = cos(pi/3)*((thruster_values["for-port-bot"]) + (-thruster_values["for-star-bot"]) + (thruster_values["aft-port-bot"]) + (-thruster_values["aft-star-bot"]) + (thruster_values["for-port-top"]) + (-thruster_values["for-star-top"]) + (thruster_values["aft-port-top"]) + (-thruster_values["aft-star-top"]))
        # net_heave = cos(pi/3)*((thruster_values["for-port-bot"]) + (thruster_values["for-star-bot"]) + (thruster_values["aft-port-bot"]) + (thruster_values["aft-star-bot"]) + (-thruster_values["for-port-top"]) + (-thruster_values["for-star-top"]) + (-thruster_values["aft-port-top"]) + (-thruster_values["aft-star-top"]))
        
        # net_pitch = cos(pi/4)*((thruster_values["for-port-bot"]) + (thruster_values["for-star-bot"]) + (-thruster_values["aft-port-bot"]) + (-thruster_values["aft-star-bot"]) + (-thruster_values["for-port-top"]) + (-thruster_values["for-star-top"]) + (thruster_values["aft-port-top"]) + (thruster_values["aft-star-top"]))
        # net_yaw = cos(pi/4)*((thruster_values["for-port-bot"]) + (-thruster_values["for-star-bot"]) + (-thruster_values["aft-port-bot"]) + (thruster_values["aft-star-bot"]) + (thruster_values["for-port-top"]) + (-thruster_values["for-star-top"]) + (-thruster_values["aft-port-top"]) + (thruster_values["aft-star-top"]))

        # net_movement_vectors = String()
        # net_movement_vectors.data = f"""Net Surge:{net_surge},Net Sway:{net_sway},Net Heave:{net_heave},Net Pitch:{net_pitch},Net Yaw:{net_yaw}"""
        # self.debugger.publish(net_movement_vectors)

        ###########################################################################################
        ############################## INIDIVISUAL THRUSTER TESITNG ###############################
        ###########################################################################################

        # Be sure to also uncomment the sef.current_thruster line in the __init__ function

        # thruster_list = ("for-port-bot", "for-star-bot", "aft-port-bot", "aft-star-bot", "for-port-top", "for-star-top", "aft-port-top", "aft-star-top")

        # if controller_inputs.open_claw:
        #     if self.current_thruster == 0:
        #         self.current_thruster = 7
        #     else:
        #         self.current_thruster -= 1
        #     self.get_logger().info(str(self.current_thruster))
        #     self.get_logger().info(str(surge))
        # elif controller_inputs.close_claw:
        #     if self.current_thruster == 7:
        #         self.current_thruster = 0
        #     else:
        #         self.current_thruster += 1
        #     self.get_logger().info(str(self.current_thruster))
        #     self.get_logger().info(str(surge))

        # for thruster_idk, channel in THRUSTER_CHANNELS.items():
        #     if channel == self.current_thruster:
        #         thruster_values[thruster_idk] = surge
        #     else:   
        #         thruster_values[thruster_idk] = 0

        
        ###########################################################################################
        ###########################################################################################
        ###########################################################################################
        return thruster_values

def main(args=None):
    rclpy.init(args=args)

    i2c_master = I2CMaster()

    rclpy.spin(i2c_master)

    i2c_master.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
