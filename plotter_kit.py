#
# Try to improve the torque and lose less steps.
#

from math import sqrt, pow, cos, sin, pi, atan
import copy, pickle, os, time, math
import numpy as np

import board
from adafruit_register.i2c_struct import UnaryStruct
from adafruit_register.i2c_struct_array import StructArray
from adafruit_bus_device import i2c_device
from micropython import const

# Constants that specify the direction and style of steps.
STEPPER_FREQ = const(2250)
SERVO_FREQ = const(50)
FORWARD = const(1)
"""Step forward"""
BACKWARD = const(2)
""""Step backward"""
SINGLE = const(1)
"""Step so that each step only activates a single coil"""
DOUBLE = const(2)
"""Step so that each step only activates two coils to produce more torque."""
INTERLEAVE = const(3)
"""Step half a step to alternate between single coil and double coil steps."""
MICROSTEP = const(4)
"""Step a fraction of a step by partially activating two neighboring coils. Step size is determined
   by ``microsteps`` constructor argument."""

class PWMChannel:
    """A single PCA9685 channel that matches the :py:class:`~pulseio.PWMOut` API."""
    def __init__(self, pca, index):
        self._pca = pca
        self._index = index

    @property
    def frequency(self):
        """The overall PWM frequency in Hertz (read-only).
        A PWMChannel's frequency cannot be set individually.
        All channels share a common frequency, set by PCA9685.frequency."""
        return self._pca.frequency

    @frequency.setter
    def frequency(self, _):
        raise NotImplementedError("frequency cannot be set on individual channels")

    @property
    def duty_cycle(self):
        """16 bit value that dictates how much of one cycle is high (1) versus low (0). 0xffff will
           always be high, 0 will always be low and 0x7fff will be half high and then half low."""
        pwm = self._pca.pwm_regs[self._index]
        if pwm[0] == 0x1000:
            return 0xffff
        return pwm[1] << 4

    @duty_cycle.setter
    def duty_cycle(self, value):
        if not 0 <= value <= 0xffff:
            raise ValueError("Out of range")

        if value == 0xffff:
            self._pca.pwm_regs[self._index] = (0x1000, 0)
        else:
            # Shift our value by four because the PCA9685 is only 12 bits but our value is 16
            value = (value + 1) >> 4
            self._pca.pwm_regs[self._index] = (0, value)

class PCAChannels: # pylint: disable=too-few-public-methods
    """Lazily creates and caches channel objects as needed. Treat it like a sequence."""
    def __init__(self, pca):
        self._pca = pca
        self._channels = [None] * len(self)

    def __len__(self):
        return 16

    def __getitem__(self, index):
        if not self._channels[index]:
            self._channels[index] = PWMChannel(self._pca, index)
        return self._channels[index]

class PCA9685:
    """
    Initialise the PCA9685 chip at ``address`` on ``i2c_bus``.
    The internal reference clock is 25mhz but may vary slightly with environmental conditions and
    manufacturing variances. Providing a more precise ``reference_clock_speed`` can improve the
    accuracy of the frequency and duty_cycle computations. See the ``calibration.py`` example for
    how to derive this value by measuring the resulting pulse widths.
    :param ~busio.I2C i2c_bus: The I2C bus which the PCA9685 is connected to.
    :param int address: The I2C address of the PCA9685.
    :param int reference_clock_speed: The frequency of the internal reference clock in Hertz.
    """
    # Registers:
    mode1_reg = UnaryStruct(0x00, '<B')
    prescale_reg = UnaryStruct(0xFE, '<B')
    pwm_regs = StructArray(0x06, '<HH', 16)

    def __init__(self, i2c_bus, *, address=0x60, reference_clock_speed=25000000):
        self.i2c_device = i2c_device.I2CDevice(i2c_bus, address)
        self.channels = PCAChannels(self)
        """Sequence of 16 `PWMChannel` objects. One for each channel."""
        self.reference_clock_speed = reference_clock_speed
        """The reference clock speed in Hz."""
        self.reset()

    def reset(self):
        """Reset the chip."""
        self.mode1_reg = 0x00 # Mode1

    @property
    def frequency(self):
        """The overall PWM frequency in Hertz."""
        return self.reference_clock_speed / 4096 / self.prescale_reg

    @frequency.setter
    def frequency(self, freq):
        prescale = int(self.reference_clock_speed / 4096.0 / freq + 0.5)
        if prescale < 3:
            raise ValueError("PCA9685 cannot output at the given frequency")
        old_mode = self.mode1_reg # Mode 1
        self.mode1_reg = (old_mode & 0x7F) | 0x10 # Mode 1, sleep
        self.prescale_reg = prescale # Prescale
        self.mode1_reg = old_mode # Mode 1
        time.sleep(0.010)
        self.mode1_reg = old_mode | 0xa1 # Mode 1, autoincrement on
        time.sleep(0.005)

    def __enter__(self):
        return self

    def __exit__(self, exception_type, exception_value, traceback):
        self.deinit()

    def deinit(self):
        """Stop using the pca9685."""
        self.reset()

class Servo:
    """Control the position of a servo.
       :param ~pulseio.PWMOut pwm_out: PWM output object.
       :param int actuation_range: The physical range of motion of the servo in degrees, \
           for the given ``min_pulse`` and ``max_pulse`` values.
       :param int min_pulse: The minimum pulse width of the servo in microseconds.
       :param int max_pulse: The maximum pulse width of the servo in microseconds.
       ``actuation_range`` is an exposed property and can be changed at any time:
        .. code-block:: python
          servo = Servo(pwm)
          servo.actuation_range = 135
       The specified pulse width range of a servo has historically been 1000-2000us,
       for a 90 degree range of motion. But nearly all modern servos have a 170-180
       degree range, and the pulse widths can go well out of the range to achieve this
       extended motion. The default values here of ``750`` and ``2250`` typically give
       135 degrees of motion. You can set ``actuation_range`` to correspond to the
       actual range of motion you observe with your given ``min_pulse`` and ``max_pulse``
       values.
       .. warning:: You can extend the pulse width above and below these limits to
         get a wider range of movement. But if you go too low or too high,
         the servo mechanism may hit the end stops, buzz, and draw extra current as it stalls.
         Test carefully to find the safe minimum and maximum.
    """
    def __init__(self, pwm_out, *, actuation_range=180, min_pulse=750, max_pulse=2250):
        self._pwm_out = pwm_out
        self.actuation_range = actuation_range
        self.set_pulse_width_range(min_pulse, max_pulse)

    def set_pulse_width_range(self, min_pulse=750, max_pulse=2250):
        """Change min and max pulse widths."""
        self._min_duty = int((min_pulse * SERVO_FREQ) / 1000000 * 0xffff)
        max_duty = (max_pulse * SERVO_FREQ) / 1000000 * 0xffff
        self._duty_range = int(max_duty - self._min_duty)

    @property
    def fraction(self):
        """Pulse width expressed as fraction between 0.0 (`min_pulse`) and 1.0 (`max_pulse`).
        For conventional servos, corresponds to the servo position as a fraction
        of the actuation range. Is None when servo is diabled (pulsewidth of 0ms).
        """
        if self._pwm_out.duty_cycle == 0:      # Special case for disabled servos
           return None
        return (self._pwm_out.duty_cycle - self._min_duty) / self._duty_range

    @fraction.setter
    def fraction(self, value):
        if value is None:
           self._pwm_out.duty_cycle = 0       # disable the motor
           return
        if not 0.0 <= value <= 1.0:
           raise ValueError("Must be 0.0 to 1.0")
        duty_cycle = self._min_duty + int(value * self._duty_range)
        self._pwm_out.duty_cycle = duty_cycle

    @property
    def angle(self):
        """The servo angle in degrees. Must be in the range ``0`` to ``actuation_range``.
        Is None when servo is disabled."""
        if self.fraction is None:  # special case for disabled servos
            return None
        return self.actuation_range * self.fraction

    @angle.setter
    def angle(self, new_angle):
        if new_angle is None:      # disable the servo by sending 0 signal
            self.fraction = None
            return
        if new_angle < 0 or new_angle > self.actuation_range:
            raise ValueError("Angle out of range")
        self.fraction = new_angle / self.actuation_range

class StepperMotor:
    """A bipolar stepper motor or four coil unipolar motor.
    :param ~pulseio.PWMOut ain1: `pulseio.PWMOut`-compatible output connected to the driver for
      the first coil (unipolar) or first input to first coil (bipolar).
    :param ~pulseio.PWMOut ain2: `pulseio.PWMOut`-compatible output connected to the driver for
      the third coil (unipolar) or second input to first coil (bipolar).
    :param ~pulseio.PWMOut bin1: `pulseio.PWMOut`-compatible output connected to the driver for
      the second coil (unipolar) or second input to second coil (bipolar).
    :param ~pulseio.PWMOut bin2: `pulseio.PWMOut`-compatible output connected to the driver for
      the fourth coil (unipolar) or second input to second coil (bipolar).
    :param int microsteps: Number of microsteps between full steps. Must be at least 2 and even.
    """
    def __init__(self, ain1, ain2, bin1, bin2, *, microsteps=8):
        self.steps_per_rev = 200
        self._coil = (ain2, bin1, ain1, bin2)
        # set a safe pwm freq for each output
        for i in range(4):
            if self._coil[i].frequency < 1500:
                self._coil[i].frequency = 2000
        self._current_microstep = 0
        if microsteps < 2:
            raise ValueError("Microsteps must be at least 2")
        if microsteps % 2 == 1:
            raise ValueError("Microsteps must be even")
        self._microsteps = microsteps
        self._curve = [int(round(0xffff * math.sin(math.pi / (2 * microsteps) * i)))
                       for i in range(microsteps + 1)]
        self._update_coils()

    def _update_coils(self, *, microstepping=False):
        duty_cycles = [0, 0, 0, 0]
        trailing_coil = (self._current_microstep // self._microsteps) % 4
        leading_coil = (trailing_coil + 1) % 4
        microstep = self._current_microstep % self._microsteps
        duty_cycles[leading_coil] = self._curve[microstep]
        duty_cycles[trailing_coil] = self._curve[self._microsteps - microstep]

        # This ensures DOUBLE steps use full torque. Without it, we'd use partial torque from the
        # microstepping curve (0xb504).
        if not microstepping and (duty_cycles[leading_coil] == duty_cycles[trailing_coil] and
                                  duty_cycles[leading_coil] > 0):
            duty_cycles[leading_coil] = 0xffff
            duty_cycles[trailing_coil] = 0xffff

        # Energize coils as appropriate:
        for i in range(4):
            self._coil[i].duty_cycle = duty_cycles[i]

    def release(self):
        """Releases all the coils so the motor can free spin, also won't use any power"""
        # De-energize coils:
        for i in range(4):
            self._coil[i].duty_cycle = 0

    def onestep(self, *, direction=FORWARD, style=SINGLE):
        """Performs one step of a particular style. The actual rotation amount will vary by style.
           `SINGLE` and `DOUBLE` will normal cause a full step rotation. `INTERLEAVE` will normally
           do a half step rotation. `MICROSTEP` will perform the smallest configured step.
           When step styles are mixed, subsequent `SINGLE`, `DOUBLE` or `INTERLEAVE` steps may be
           less than normal in order to align to the desired style's pattern.
           :param int direction: Either `FORWARD` or `BACKWARD`
           :param int style: `SINGLE`, `DOUBLE`, `INTERLEAVE`"""
        # Adjust current steps based on the direction and type of step.
        step_size = 0
        if style == MICROSTEP:
            step_size = 1
        else:
            half_step = self._microsteps // 2
            full_step = self._microsteps
            # Its possible the previous steps were MICROSTEPS so first align with the interleave
            # pattern.
            additional_microsteps = self._current_microstep % half_step
            if additional_microsteps != 0:
                # We set _current_microstep directly because our step size varies depending on the
                # direction.
                if direction == FORWARD:
                    self._current_microstep += half_step - additional_microsteps
                else:
                    self._current_microstep -= additional_microsteps
                step_size = 0
            elif style == INTERLEAVE:
                step_size = half_step
            current_interleave = self._current_microstep // half_step
            if ((style == SINGLE and current_interleave % 2 == 1) or
                    (style == DOUBLE and current_interleave % 2 == 0)):
                step_size = half_step
            elif style in (SINGLE, DOUBLE):
                step_size = full_step
        if direction == FORWARD:
            self._current_microstep += step_size
        else:
            self._current_microstep -= step_size
        # Now that we know our target microstep we can determine how to energize the four coils.
        self._update_coils(microstepping=style == MICROSTEP)
        return self._current_microstep

class PlotterKit:
    """
    A flatter structure which re-uses the PCA9685 object
    and moves the frequency around to take advantage of steppers and
    servos at the same time to provide the pen lifter.
    """
    def __init__(self, address=0x60, i2c=None, steppers_microsteps=8):
        self._stepper1 = None
        self._stepper2 = None
        self._servo = None
        if i2c is None:
            i2c = board.I2C()
        self._pca = PCA9685(i2c, address=address)
        self.mode = 'stepper'
        self.stepper_freq = STEPPER_FREQ
        self._pca.frequency = self.stepper_freq
        self.servo_freq = SERVO_FREQ
        self._steppers_microsteps = steppers_microsteps

    def switch_mode(self, mode):
        if (mode == 'servo' and self.mode != 'servo'):
            self._pca.frequency = self.servo_freq
            self.mode = 'servo'
        if (mode == 'stepper' and self.mode != 'stepper'):
            self._pca.frequency = self.stepper_freq
            self.mode = 'stepper'

    def release(self):
        self.switch_mode('stepper')
        self._stepper1.release()
        self._stepper2.release()
        return

    @property
    def servo(self):
        if (self.mode == 'stepper'):
            self.switch_mode('servo')
        if not self._servo:
            self._servo = Servo(self._pca.channels[15], actuation_range=160, min_pulse=750, max_pulse=2250)
        return self._servo

    @property
    def stepper1(self):
        """:py:class:``~adafruit_motor.stepper.StepperMotor`` controls for one connected to stepper
           1 (also labeled motor 1 and motor 2).
            The following image shows the location of the stepper1 terminals on the DC/Stepper
            FeatherWing. stepper1 is made up of the M1 and M2 terminals.
            The labels on the FeatherWing are found on the bottom of the board.
            The terminals are labeled on the top of the Shield and Pi Hat.
            .. image :: ../docs/_static/motor_featherwing/stepper1.jpg
              :alt: Stepper 1 location
            This example moves the stepper motor 100 steps forwards.
            .. code-block:: python
                from adafruit_motorkit import MotorKit
                kit = MotorKit()
                for i in range(100):
                    kit.stepper1.onestep()
        """
        if (self.mode == 'servo'):
            self.switch_mode('stepper')
        if not self._stepper1:
            from adafruit_motor import stepper
            self._pca.channels[8].duty_cycle = 0xffff
            self._pca.channels[13].duty_cycle = 0xffff
            self._stepper1 = StepperMotor(self._pca.channels[10], self._pca.channels[9],
                                                  self._pca.channels[11], self._pca.channels[12],
                                                  microsteps=self._steppers_microsteps)
        return self._stepper1

    @property
    def stepper2(self):
        """:py:class:``~adafruit_motor.stepper.StepperMotor`` controls for one connected to stepper
           2 (also labeled motor 3 and motor 4).
            The following image shows the location of the stepper2 terminals on the DC/Stepper
            FeatherWing. stepper2 is made up of the M3 and M4 terminals.
            The labels on the FeatherWing are found on the bottom of the board.
            The terminals are labeled on the top of the Shield and Pi Hat.
            .. image :: ../docs/_static/motor_featherwing/stepper2.jpg
              :alt: Stepper 2 location
            This example moves the stepper motor 100 steps forwards.
            .. code-block:: python
                from adafruit_motorkit import MotorKit
                kit = MotorKit()
                for i in range(100):
                    kit.stepper2.onestep()
        """
        if (self.mode == 'servo'):
            self.switch_mode('stepper')
        if not self._stepper2:
            from adafruit_motor import stepper
            self._pca.channels[7].duty_cycle = 0xffff
            self._pca.channels[2].duty_cycle = 0xffff
            self._stepper2 = StepperMotor(self._pca.channels[4], self._pca.channels[3],
                                                  self._pca.channels[5], self._pca.channels[6],
                                                  microsteps=self._steppers_microsteps)
        return self._stepper2
