from enum import Enum
import array
import numpy as np
from collections import deque
import scipy.spatial.transform as transform
from geometry_msgs.msg import PoseArray, Pose  # ROS2 classes

RPM_TO_RAD_S = 0.10472 # mutiply with a rpm value to get the rad/s value
RPM_TO_DEG_S = 6 # mutiply with a rpm value to get the °/s value

class ArmLeg(Enum):
    """
        Enum class to define the arm leg side. For code readability, the arm/leg side can be defined with this enum.
    """
    ARM = 0
    LEG = 1
    WHEEL = 2

class ArmLegSide(Enum):
    """
        Enum class to define the arm/leg side. It corresponds to the arm_id/leg_id in the joint values received by the robot.
    """
    NONE = 0
    LEFT = 1
    RIGHT = 2

class ArticulationDesc:
    """
    Descriptor class for the articulation attributes of the robot controller. It checks the type and the length of the list.
    """
    def __set_name__(self, owner, name):
        self.name = name

    def __get__(self, instance, owner):
        if instance is None:
            return self
        return instance.__dict__.get(f"_{id(instance)}_{self.name}", [0.0] * instance.arm_joints_number)

    def __set__(self, instance, value):
        if isinstance(value, array.array) and value.typecode == 'd':
            instance.__dict__[f"_{id(instance)}_{self.name}"] = list(value)
            return
        if not isinstance(value, list) or not all(isinstance(i, float) for i in value):
            raise TypeError(f"{self.name} must be a list/array of floats, got {value} of type {type(value)}")
        if len(value) != instance.leg_joints_number and len(value) != instance.arm_joints_number:
            raise ValueError(f"{self.name} must have {instance.leg_joints_number} or {instance.arm_joints_number} elements ({len(value)} given)")
        instance.__dict__[f"_{id(instance)}_{self.name}"] = value

    def __str__(self) -> str:
        return str(self.__get__(self, self.__class__))

    
class JointAnglesValues:
    """
    
    """
    @property
    def arm_limb_number(self):
        return self._limb_number[ArmLeg.ARM.value]
    @arm_limb_number.setter
    def arm_limb_number(self, value):
        self._limb_number[ArmLeg.ARM.value] = value
    @property
    def leg_limb_number(self):
        return self._limb_number[ArmLeg.LEG.value]
    @leg_limb_number.setter
    def leg_limb_number(self, value):
        self._limb_number[ArmLeg.LEG.value] = value
    @property
    def wheel_limb_number(self):
        return self._limb_number[ArmLeg.WHEEL.value]
    @wheel_limb_number.setter
    def wheel_limb_number(self, value):
        self._limb_number[ArmLeg.WHEEL.value] = value
    @property
    def limb_number(self, arm_leg: ArmLeg):
        return self._limb_number[arm_leg.value]
    
    def set_limb_number(self, arm_leg: ArmLeg, value):
        self._limb_number[arm_leg.value] = value

    @property
    def arm_joints_number(self):
        return self._joints_number[ArmLeg.ARM.value]
    @arm_joints_number.setter
    def arm_joints_number(self, value):
        self._joints_number[ArmLeg.ARM.value] = value
    @property
    def leg_joints_number(self):
        return self._joints_number[ArmLeg.LEG.value]
    @leg_joints_number.setter
    def leg_joints_number(self, value):
        self._joints_number[ArmLeg.LEG.value] = value
    @property
    def joints_number(self, arm_leg: ArmLeg):
        return self._joints_number[arm_leg.value]
    
    def set_joints_number(self, arm_leg: ArmLeg, value):
        self._joints_number[arm_leg.value] = value

    @property
    def wheeled(self):
        return self._wheeled
    @wheeled.setter
    def wheeled(self, value):
        self._wheeled = value


    @property
    def default_limb(self):
        return self._default_limb
    @default_limb.setter
    def default_limb(self, value: ArmLeg):
        self._default_limb = value

    @property
    def default_side(self):
        return self._default_side
    @default_side.setter
    def default_side(self, value: ArmLegSide):
        self._default_side = value

    @property
    def value(self):
        """
            Get the joint values for the default limb and side.
        """
        if self._default_limb is None:# or not isinstance(self._default_limb, ArmLeg):
            raise AttributeError("The default limb is not set")
        if self._default_side is None:# or not isinstance(self._default_side, ArmLegSide):
            raise AttributeError("The default side is not set")
        
        return self._data[self._default_limb.value][self._default_side.value]
    
    @value.setter
    def value(self, value):
        """
            Set the joint values for the default limb and side.
        """
        if self._default_limb is None:
            raise AttributeError("The default limb is not set")
        if self._default_side is None:
            raise AttributeError("The default side is not set")
        
        self.set_limb_side_articulations_value(self._default_limb, self._default_side, value)


    _temp_left_pos = ArticulationDesc()
    _temp_right_pos = ArticulationDesc()    

    def __init__(self) -> None:
        self._limb_number = {}
        self._joints_number = {}
        self.wheeled = False
        self._default_side = None
        self._default_limb = None

        self._data = {
        ArmLeg.ARM.value: 
            {
                ArmLegSide.LEFT.value: None,
                ArmLegSide.RIGHT.value: None
            },
        ArmLeg.LEG.value:
            {
                ArmLegSide.LEFT.value: None,
                ArmLegSide.RIGHT.value: None
            },
        ArmLeg.WHEEL.value:
            {
            }
    }

    def set_limb_articulations_value(self, arm_leg: ArmLeg, value):
        """
            Set the joint values for the given limb and side.
        """       
        temp_data = {
            ArmLegSide.LEFT.value: None,
            ArmLegSide.RIGHT.value: None
        }
            
        if value is not None and len(value) == self._joints_number[arm_leg.value] * self._limb_number[arm_leg.value]:
            # If home position contain _JOINTS_NUMBER * _LIMB_NUMBER elements, the home position is different for each limb
            # The first _JOINTS_NUMBER elements are for the left limb, the next _JOINTS_NUMBER elements are for the right limb
            self._temp_left_pos = value[:self._joints_number[arm_leg.value]]
            temp_data[ArmLegSide.LEFT.value] = self._temp_left_pos
            if self._limb_number[arm_leg.value] == 2:
                self._temp_right_pos = value[self._joints_number[arm_leg.value]:]
                temp_data[ArmLegSide.RIGHT.value] = self._temp_right_pos
            else:
                # If a home position is given for both side, but only one limb is defined, the home position is set to None for the right limb
                temp_data[ArmLegSide.RIGHT.value] = None
        elif value is not None and len(value) == self._joints_number[arm_leg.value]:
            # If home position contain only _JOINTS_NUMBER elements (and have more than one limb), the home position is the same for both
            self._temp_left_pos = value
            temp_data[ArmLegSide.LEFT.value] = self._temp_left_pos
            temp_data[ArmLegSide.RIGHT.value] = self._temp_left_pos
        else:
            # If no home position is set, set to None (shouldn't happen if _LIMIT_NUMBER > 0)
            temp_data[ArmLegSide.LEFT.value] = None
            temp_data[ArmLegSide.RIGHT.value] = None

        self._data[arm_leg.value] = temp_data

    def set_limb_side_articulations_value(self, arm_leg: ArmLeg, side: ArmLegSide, value: list):
        """
            Set the joint values for the given limb and side.
        """
        if value is not None and len(value) == self._joints_number[arm_leg.value]:
            self._data[side.value] = value
        else:
            raise ValueError(f"Invalid joint values for the {side.name} {arm_leg.name} limb: {value}")
        
    def __str__(self):
        new_data = {}
        enum_classes = (ArmLeg, ArmLegSide)
    
        # Iterate over the outer dictionary
        for outer_key, inner_dict in self._data.items():
            # Find the corresponding enum name for the outer key
            outer_key_name = next(e.name for e in enum_classes[0] if e.value == outer_key)
            
            # Replace keys in the inner dictionary
            new_inner_dict = {}
            try:
                for inner_key, value in inner_dict.items():
                    inner_key_name = next(e.name for e in enum_classes[1] if e.value == inner_key)
                    new_inner_dict[inner_key_name] = value
            except AttributeError: # If the inner_dict is empty
                pass

            # Update the outer dictionary with the replaced keys
            new_data[outer_key_name] = new_inner_dict
        
        return str(new_data)
    
    def __getitem__(self, key):
        if hasattr(key, 'value') and key.value in self._data:
            return self._data[key.value]
        if isinstance(key, str) and key.upper() in ArmLeg.__members__:
            return self._data[ArmLeg[key.upper()].value]
        return AttributeError(f"Invalid key: {key}")

    def __setitem__(self, key, value):
        if hasattr(key, 'value') and key.value in self._data:
            if isinstance(value, dict):
                self._data[key.value] = value
            elif isinstance(value, list):
                self.set_limb_articulations_value(key, value)
            else:
                raise ValueError(f"Invalid value for key {key}: {value}")
        elif isinstance(key, str) and key.upper() in ArmLeg.__members__:
            self.set_limb_articulations_value(ArmLeg[key.upper()], value)
        else:
            raise AttributeError(f"Invalid key: {key}")
        
    def __iter__(self):
        # Get an iterator over the items in `_data`, which is a dictionary
        self._data_iter = iter(self._data.items())
        return self

    def __next__(self):
        # Use `next()` on the iterator to get the next item
        try:
            return next(self._data_iter)
        except StopIteration:
            # Raise StopIteration to end iteration when there are no more items
            raise StopIteration
    

class DataBuffer(deque):
    """
        A deque subclass that holds a fixed number of items. When a new item is added and the buffer is full, the oldest item is removed.
        Should only use with numpy arrays.
    """
    def __init__(self, max_len: int) -> None:
        super().__init__(maxlen=max_len)

    def append(self, item):
        if not isinstance(item, np.ndarray):
            raise TypeError("Only numpy arrays are allowed")
        super().append(item)

    def appendleft(self, x):
        if not isinstance(x, np.ndarray):
            raise TypeError("Only numpy arrays are allowed")
        return super().appendleft(x)

    def extend(self, items):
        if not all(isinstance(item, np.ndarray) for item in items):
            raise TypeError("Only numpy arrays are allowed")
        super().extend(items)

    def insert(self, i, x):
        if not isinstance(x, np.ndarray):
            raise TypeError("Only numpy arrays are allowed")
        return super().insert(i, x)
    
    def is_full(self):
        return len(self) == self.maxlen
    
    def is_empty(self):
        return len(self) == 0
    
    def last_append(self):
        return self[-1]
    
    def mean(self, full_only=False):
        if full_only and not self.is_full():
            return None
        return np.mean(np.stack(self, axis=0), axis=0)
    
    def median(self, full_only=False):
        if full_only and not self.is_full():
            return None
        return np.median(np.stack(self, axis=0), axis=0)
    
    def lowpass_filter(self, alpha=0.5):
        """
            Apply a low-pass filter to the data buffer.
        """
        if not self.is_full():
            return None
        return alpha * self[-1] + (1 - alpha) * self.mean(full_only=True)


class EndEffectorCoordinates:
    @property
    def position(self):
        return self._position
    
    @property
    def orientation(self):
        return self._orientation

    @property
    def coordinates(self):
        return np.concatenate((self._position, self._orientation))
    
    @coordinates.setter
    def coordinates(self, value):
        self.from_list(value)

    @position.setter
    def position(self, value):
        if len(value) != 3:
            raise ValueError(f"Invalid position: {value}")
        self._position = np.array(value)

    @orientation.setter
    def orientation(self, value):
        if len(value) not in [3, 4]:
            raise ValueError(f"Invalid orientation: {value}")
        self._orientation = np.array(value)

    @property
    def x(self):
        return self._position[0]
    @x.setter
    def x(self, value):
        self._position[0] = value

    @property
    def y(self):
        return self._position[1]
    @y.setter
    def y(self, value):
        self._position[1] = value

    @property
    def z(self):
        return self._position[2]
    @z.setter
    def z(self, value):
        self._position[2] = value

    @property
    def roll(self):
        return self._orientation[0] if len(self._orientation) == 3 else self.quaternion_to_euler()[0]
    @roll.setter
    def roll(self, value):
        if len(self._orientation) == 3:
            self._orientation[0] = value

    @property
    def pitch(self):
        return self._orientation[1] if len(self._orientation) == 3 else self.quaternion_to_euler()[1]
    @pitch.setter
    def pitch(self, value):
        if len(self._orientation) == 3:
            self._orientation[1] = value

    @property
    def yaw(self):
        return self._orientation[2] if len(self._orientation) == 3 else self.quaternion_to_euler()[2]
    @yaw.setter
    def yaw(self, value):
        if len(self._orientation) == 3:
            self._orientation[2] = value

    @property
    def qx(self):
        return self._orientation[0] if len(self._orientation) == 4 else self.euler_to_quaternion()[0]
    @qx.setter
    def qx(self, value):
        if len(self._orientation) == 4:
            self._orientation[0] = value

    @property
    def qy(self):
        return self._orientation[1] if len(self._orientation) == 4 else self.euler_to_quaternion()[1]
    @qy.setter
    def qy(self, value):
        if len(self._orientation) == 4:
            self._orientation[1] = value

    @property
    def qz(self):
        return self._orientation[2] if len(self._orientation) == 4 else self.euler_to_quaternion()[2]
    @qz.setter
    def qz(self, value):
        if len(self._orientation) == 4:
            self._orientation[2] = value

    @property
    def qw(self):
        return self._orientation[3] if len(self._orientation) == 4 else self.euler_to_quaternion()[3]
    @qw.setter
    def qw(self, value):
        if len(self._orientation) == 4:
            self._orientation[3] = value
    
    def __init__(self, value=None) -> None:
        if value is not None:
            self.from_list(value)
        else:
            self.zeros()

    def from_list(self, value):
        if len(value) == 6:
            self._position = np.array(value[:3])
            self._orientation = np.array(value[3:])
        elif len(value) == 7:
            self._position = np.array(value[:3])
            self._orientation = np.array(value[3:])
        else:
            raise ValueError(f"Invalid coordinates: {value}")

    def fill(self, value):
        self._position = np.full(3, value)
        self._orientation = np.full(4 if len(self._orientation) == 4 else 3, value)

    def zeros(self):
        self._position = np.zeros(3)
        self._orientation = np.array([0.0, 0.0, 0.0, 1.0]) if len(self._orientation) == 4 else np.zeros(3)

    def quaternion_to_euler(self):
        if len(self._orientation) == 4:
            rot = transform.Rotation.from_quat(self._orientation)
            return rot.as_euler('xyz')
        else:
            return self._orientation

    def euler_to_quaternion(self):
        if len(self._orientation) == 3:
            rot = transform.Rotation.from_euler('xyz', self._orientation)
            return rot.as_quat()
        else:
            return self._orientation

    def to_list(self):
        return self.coordinates.tolist()

    def __str__(self):
        return f"Position: {self._position}, Orientation: {self._orientation}"

    def __getitem__(self, key):
        if isinstance(key, int):
            if key < 3:
                return self._position[key]
            elif key < len(self.coordinates):
                return self._orientation[key - 3]
            else:
                raise IndexError(f"Index out of range: {key}")
        elif key == 'position':
            return self._position
        elif key == 'orientation':
            return self._orientation
        else:
            raise AttributeError(f"Invalid key: {key}")

    def __setitem__(self, key, value):
        if isinstance(key, int):
            if key < 3:
                self._position[key] = value
            elif key < len(self.coordinates):
                self._orientation[key - 3] = value
            else:
                raise IndexError(f"Index out of range: {key}")
        elif key == 'position':
            self._position = np.array(value)
        elif key == 'orientation':
            if len(value) not in [3, 4]:
                raise ValueError(f"Invalid orientation: {value}")
            self._orientation = np.array(value)
        else:
            raise AttributeError(f"Invalid key: {key}")
        
    def to_pose_msg(self):
        pose = Pose()
        pose.position.x, pose.position.y, pose.position.z = self._position
        pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w = self._orientation if len(self._orientation) == 4 else self.euler_to_quaternion()
        return pose

    def __iter__(self):
        return iter(self._position)

    def __next__(self):
        return next(self._position)

    def __len__(self):
        return len(self._position)

    def __eq__(self, other):
        return self._position == other._position and self._orientation == other._orientation

    def __ne__(self, other):
        return not self.__eq__(other)

    def __add__(self, other):
        return np.add(self.coordinates, other.coordinates)

    def __sub__(self, other):
        return np.subtract(self.coordinates, other.coordinates)

    def __mul__(self, other):
        return np.multiply(self.coordinates, other.coordinates)

    def __truediv__(self, other):
        return np.divide(self.coordinates, other.coordinates)

    def __floordiv__(self, other):
        return np.floor_divide(self.coordinates, other.coordinates)

    def __mod__(self, other):
        return np.mod(self.coordinates, other.coordinates)

    def __pow__(self, other):
        return np.power(self.coordinates, other.coordinates)

    def __abs__(self):
        return np.abs(self.coordinates)

    def __neg__(self):
        return np.negative(self.coordinates)

    def __pos__(self):
        return np.positive(self.coordinates)

    def __invert__(self):
        return np.invert(self.coordinates)

    def __lshift__(self, other):
        return np.left_shift(self.coordinates, other.coordinates)

    def __rshift__(self, other):
        return np.right_shift(self.coordinates, other.coordinates)

   