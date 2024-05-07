import rclpy
from rclpy.node import Node, MutuallyExclusiveCallbackGroup

from eer_messages.msg import ThrusterMultipliers, PilotInput, IMUData, ADCData, TempSensorData, OutsideTempProbeData

import board
from smbus2 import SMBus
from adafruit_pca9685 import PCA9685
from adafruit_bno055 import BNO055_I2C

from math import sqrt

# Configure minimum, middle and maximum pulse lengths (out of 4096)
# Values should be adjusted so that the center arms the thrusters
MIN_SPEED = 1000
MAX_SPEED = 1500
CENTER_SPEED = 1250

THUSTER_ACCELERATION = 75


# Thurster channels are based on Beaumont as of May 7th 2024
THRUSTER_CHANNELS = {
    "for-port-top": 1,
    "for-star-top": 0,
    "aft-port-top": 2,
    "aft-star-top": 7,
    "for-port-bot": 4,
    "for-star-bot": 6,
    "aft-port-bot": 5,
    "aft-star-bot": 3
}

THRUSTER_TICK_RATE = 0.01

STM32_ADDRESS = 0x80 # Don't know yet

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
    def __init__(self, pwm, ch):
        """
        Setup the thruster or servo.

        :param pwm: I2C PWM driver controller object
        :param ch: pwm channel number for the thruster or servo controller
        """
        self.pwm = pwm
        self.ch = ch

        self.target = CENTER_SPEED
        self.current = CENTER_SPEED

        # Arm thruster
        pwm.channels[ch].duty_cycle = CENTER_SPEED

    def thruster_scale(self, thruster):
        """
        Scale thruster speed(-1.0 to 1.0) to pwm min/center/max limits.

        :param thruster: thruster to scale
        """
        if thruster >= 0:
            t = int(CENTER_SPEED + (MAX_SPEED - CENTER_SPEED) * thruster)
        else:
            t = int(CENTER_SPEED + (CENTER_SPEED - MIN_SPEED) * thruster)
        return t

    def fly(self, speed):
        """
        Drive thrusters using given speed parameters.

        :param speed: speed of thrusters with valid range between -1.0 to 1.0
        """
        self.target = self.thruster_scale(speed)

    def tick(self):
        if(self.current != self.target):
            if(abs(self.target - self.current) > THUSTER_ACCELERATION):
                direction = (self.target - self.current) / abs(self.target - self.current)
                self.current += int(direction * THUSTER_ACCELERATION)
                self.pwm.channels[self.ch].duty_cycle = self.current
            else:
                self.current = self.target
                self.pwm.channels[self.ch].duty_cycle = self.current

class I2CMaster(Node):
    '''
    This class will communicate to multiple devices on the same I2C bus. These include the following:

        1. PCA9685: Talks to the 8 Thrusters connected to the bot. Uses the adafruit_pca9685 library.
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
        
        ###############################################################
        ################### INITIALIZE ADAFRUIT I2C ###################
        ###############################################################        

        self.i2c = None

        try:
            self.i2c = board.I2C()
        except:
            self.get_logger().error("No Hardware on I2C bus")

        #################################################
        ################### THRUSTERS ###################
        #################################################

        self.power_multiplier = 0.2
        self.surge_multiplier = 0
        self.sway_multiplier = 0
        self.heave_multiplier = 0
        self.pitch_multiplier = 0
        self.yaw_multiplier = 0

        self.pca9685_detected = False

        self.connected_channels = {}

        if self.i2c is not None:

            # Connect to PCA9685
            try:
                self.pwm = PCA9685(self.i2c)         
                self.pwm.frequency = 100
                self.get_logger().info("PCA DETECTED ON I2C BUS")
                
                for thuster in THRUSTER_CHANNELS:  
                    self.connected_channels[THRUSTER_CHANNELS[thuster]] = Thruster(self.pwm, THRUSTER_CHANNELS[thuster])

                self.pca9685_detected = True 
                
                self.thruster_timer = self.create_timer(THRUSTER_TICK_RATE, self.tick_thrusters, callback_group=i2c_master_callback_group)

            except:
                self.get_logger().error("CANNOT FIND PCA9685 ON I2C BUS!")


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
                configuration_value = 0b00100000 # (auto conversion mode)

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

        if self.pca9685_detected:
            
            for thruster_position in THRUSTER_CHANNELS:
                #self.get_logger().info(f"{thruster_position}: {thruster_values[thruster_position]}")
                self.connected_channels[THRUSTER_CHANNELS[thruster_position]].fly(thruster_values[thruster_position])
            

        # self.stm32_communications(msg)

    def stm32_communications(self, controller_inputs):
        '''
        Communications to the STM32 is required for the 12 DC motor (claw linear actuator),
        stepper motor (claw rotation), dimmable LEDs, and outside temperature probe readings.

        The eer properity communication protocol is (currently) based only on reads. Each action will be achieved through:

        read = self.bus.read_byte_data(STM32_ADDRESS, action_address)

        The result of the read can be interperted for a success code (ex. 0xFF) or a fail code (0xF0). A non successful transaction 
        (where the slave doesn't respond or acknowledge) results in SMBus returning 0x00. 

        An exception to the read result above is the temperature reading from the 1-wire outside probe. That will be delieved in two 
        bytes in the same format it is recieved in the STM32 by the 1 wire protocol (https://cdn.sparkfun.com/datasheets/Sensors/Temp/DS18B20.pdf).  

        Thus, possible actions and associated registers are as follows:

        0x11 - open_claw
        0x12 - close_claw
        0x13 - brighten_led
        0x14 - dim_led
        0x15 - turn_claw_cw
        0x16 - turn_claw_ccw
        0x17 - read_outside_temperature_probe
        '''
        
        if self.bus is not None:

            possible_actions = {
                0x11: controller_inputs.open_claw,
                0x12: controller_inputs.close_claw,
                0x13: controller_inputs.brighten_led,
                0x14: controller_inputs.dim_led,
                0x15: controller_inputs.turn_claw_cw,
                0x16: controller_inputs.turn_claw_ccw,
            }

            outside_temperature_probe_register = 0x17
            temperature_probe_register_length_in_bytes = 2

            for action_address, is_desired in possible_actions.items():
                if is_desired:
                    read = self.bus.read_byte_data(STM32_ADDRESS, action_address)

                    if read == 0xF0:
                        self.get_logger().error(f"COULD NOT PERFORM ACTION WITH ACTION ADDRESS {action_address}")
                    elif read == 0x00:
                        self.get_logger().error(f"COULD NOT COMMUNICATE TO STM32")
            
            if controller_inputs.read_outside_temperature_probe:
                    
                outside_temp_probe_data = OutsideTempProbeData()

                # Data will arrive based on this format (https://cdn.sparkfun.com/datasheets/Sensors/Temp/DS18B20.pdf)

                read = self.bus.read_i2c_block_data(STM32_ADDRESS, outside_temperature_probe_register, temperature_probe_register_length_in_bytes)

                # Convert from 12 bit two's complement binary to string
                read_12_bit_twos_complement_str = '{0:016b}'.format((read[0] << 8) + read[1])[4:]

                # Convert from 12 bit two's complement string to signed binary

                read_12_bit_twos_complement = int(f"0b{read_12_bit_twos_complement_str}",2)

                read_12_bit_magnitude_str = '{0:016b}'.format(((0b111111111111 - read_12_bit_twos_complement) + 1) if read_12_bit_twos_complement >= 0b100000000000 else read_12_bit_twos_complement)[4:] 

                # Convert to float32

                outside_temp_probe_data.data = (-1 if int(read_12_bit_twos_complement_str[0]) else 1) * (int(f"0b{read_12_bit_magnitude_str[:8]}",2) + 0.5 * int(read_12_bit_magnitude_str[8]) + 0.25 * int(read_12_bit_magnitude_str[9]) + 0.125 * int(read_12_bit_magnitude_str[10]) + 0.0625 * int(read_12_bit_magnitude_str[11]))

                if outside_temp_probe_data.data == 0xF0:
                    self.get_logger().error(f"COULD NOT PERFORM ACTION WITH ACTION ADDRESS {bin(outside_temperature_probe_register)}")
                elif outside_temp_probe_data.data == 0x00:
                    self.get_logger().error(f"COULD NOT COMMUNICATE TO STM32")

                self.outside_temperature_probe_data_publisher.publish(outside_temp_probe_data)

                
                


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

        # Calculations below are based on thruster positions
        thruster_values["for-port-bot"] = ((-surge)+(sway)+(heave)) * combined_strafe_coefficient + ((pitch)+(yaw)) * rotation_average_coefficient
        thruster_values["for-star-bot"] = ((-surge)+(-sway)+(heave)) * combined_strafe_coefficient + ((pitch)+(-yaw)) * rotation_average_coefficient
        thruster_values["aft-port-bot"] = ((surge)+(sway)+(heave)) * combined_strafe_coefficient + ((-pitch)+(-yaw)) * rotation_average_coefficient
        thruster_values["aft-star-bot"] = ((surge)+(-sway)+(heave)) * combined_strafe_coefficient + ((-pitch)+(yaw)) * rotation_average_coefficient
        thruster_values["for-port-top"] = ((-surge)+(sway)+(-heave)) * combined_strafe_coefficient + ((-pitch)+(yaw)) * rotation_average_coefficient
        thruster_values["for-star-top"] = ((-surge)+(-sway)+(-heave)) * combined_strafe_coefficient + ((-pitch)+(-yaw)) * rotation_average_coefficient
        thruster_values["aft-port-top"] = ((surge)+(sway)+(-heave)) * combined_strafe_coefficient + ((pitch)+(-yaw)) * rotation_average_coefficient
        thruster_values["aft-star-top"] = ((surge)+(-sway)+(-heave)) * combined_strafe_coefficient + ((pitch)+(yaw)) * rotation_average_coefficient

        ##################################################################################################################
        ############################## FINIDING CENTER SPEED, TESTING INDIVISUAL THRUSTERS ###############################
        ##################################################################################################################

        # global MAX_SPEED, CENTER_SPEED, MIN_SPEED

        # thrusters = ("for-port-bot", "for-star-bot", "aft-port-bot","aft-star-bot", "for-port-top","for-star-top","aft-port-top","aft-star-top")

        # if controller_inputs.brighten_led:
        #     MAX_SPEED += 100
        #     CENTER_SPEED += 100
        #     MIN_SPEED += 100
        #     self.get_logger().info(str([MIN_SPEED, CENTER_SPEED, MAX_SPEED]))
        
        # if controller_inputs.dim_led:
        #     MAX_SPEED -= 100
        #     CENTER_SPEED -= 100
        #     MIN_SPEED -= 100
        #     self.get_logger().info(str([MIN_SPEED, CENTER_SPEED, MAX_SPEED]))

        # if controller_inputs.open_claw:
        #     if self.current_thruster == 7:
        #         self.current_thruster = 0
        #     else:
        #         self.current_thruster += 1

        # if controller_inputs.close_claw:
        #     if self.current_thruster == 0:
        #         self.current_thruster = 7
        #     else:
        #         self.current_thruster -= 1

        # thruster_values[thrusters[self.current_thruster]] = ((surge)+(-sway)+(-heave)) * combined_strafe_coefficient + ((pitch)+(yaw)) * rotation_average_coefficient


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

        
        ####################################################################
        ####################################################################
        ####################################################################

        return thruster_values

def main(args=None):
    rclpy.init(args=args)

    i2c_master = I2CMaster()

    rclpy.spin(i2c_master)

    i2c_master.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
