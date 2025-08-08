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
    MIDDLE = 3
    LEFT2 = 4
    RIGHT2 = 5

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
    
    def min_elements(self, other: 'EndEffectorCoordinates') -> 'EndEffectorCoordinates':
        """
            Returns a new EndEffectorCoordinates object whose coordinates are the element-wise minimum 
            between this object and another.

            Args:
                other (EndEffectorCoordinates): Another instance to compare with.

            Returns:
                EndEffectorCoordinates: A new instance with element-wise minimum coordinates.

            Raises:
                ValueError: If the orientation lengths between the two objects do not match, indicating
                            inconsistent orientation types (e.g., Euler vs Quaternion).
        """
        if len(self.orientation) != len(other.orientation):
            raise ValueError("Orientations are not consistent (Euler/Quaternion) between both objects")
        ans = EndEffectorCoordinates()
        for i in range(len(self.coordinates)):
            ans.coordinates[i] = min(self.coordinates[i], other.coordinates[i])

        return ans
    
    def max_elements(self, other: 'EndEffectorCoordinates') -> 'EndEffectorCoordinates':
        """
            Returns a new EndEffectorCoordinates object whose coordinates are the element-wise maximum 
            between this object and another.

            Args:
                other (EndEffectorCoordinates): Another instance to compare with.

            Returns:
                EndEffectorCoordinates: A new instance with element-wise maximum coordinates.

            Raises:
                ValueError: If the orientation lengths between the two objects do not match, indicating
                            inconsistent orientation types (e.g., Euler vs Quaternion).
        """
        if len(self.orientation) != len(other.orientation):
            raise ValueError("Orientations are not consistent (Euler/Quaternion) between both objects")
        ans = EndEffectorCoordinates()
        for i in range(len(self.coordinates)):
            ans.coordinates[i] = max(self.coordinates[i], other.coordinates[i])

        return ans
    
    def el_name(self, el):
        if len(self._orientation) == 3:
            names = ["x", "y", "z", "roll", "pitch", "yaw"]
        else:
            names = ["x", "y", "z", "qx", "qy", "qz", "qw"]
        return names[el] 

    def __iter__(self):
        return iter(self.coordinates)

    def __next__(self):
        return next(self.coordinates)

    def __len__(self):
        return len(self.coordinates)

    def __eq__(self, other):
        return np.allclose(self._position, other._position) and np.allclose(self._orientation, other._orientation)

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


class LimbData:
    @property
    def empty(self):
        return self._empty
    
    @property
    def data_type(self):
        return self._data_type
    
    @property
    def config(self):
        return list(self._data.keys())

    def __init__(self, config, default_value=None, force_same_value_type=False):
        """
            Initializes a LimbData object with a given configuration of limbs.

            Args:
                config (list of tuples or str): A list of (ArmLeg, ArmLegSide) tuples specifying the limbs to configure,
                    or a string representing a predefined configuration (e.g., "humanoid", "dual arm").
                default_value (any, optional): The value to assign to each limb initially. Defaults to None.
                force_same_value_type (bool, optional): If True, enforces that all values assigned to limbs
                    must be of the same type as default_value. Raises a TypeError on mismatched assignments.

            Raises:
                ValueError: If force_same_value_type is True and default_value is None.
        """
        self._empty = True
        if isinstance(config, str):
            config = LimbData._predefined_config(config)

        if force_same_value_type:
            if default_value is None:
                raise ValueError("Default value cannot be None if force_same_value_type is True.")
            self._data_type = type(default_value)
        else:
            self._data_type = None

        self._data = {}
        for limb_type, side in config:
            self.add_limb(limb_type, side, default_value)

    def add_limb(self, limb_type, side, value):
        self.check_data(value)
        
        self._data[(limb_type, side)] = value
        self._empty = False

    def add_limbs(self, config, value):
        if isinstance(config, str):
            config = LimbData._predefined_config(config)

        if isinstance(value, list):
            if len(value) != len(config):
                raise ValueError("List of values should be the same length as the config list")
            
            for (limb_type, side), val in zip(config, value):
                self.add_limb(limb_type, side, val)
        else:
            for limb_type, side in config:
                self.add_limb(limb_type, side, value)

    def check_data(self, value):
        if self._data_type is not None and not isinstance(value, self._data_type):
            raise TypeError(f"Value should be {self._data_type}, but {type(value)} has been received.")

    def set(self, limb_type, side, value):
        self.check_data(value)
        
        key = (limb_type, side)
        if key not in self._data:
            raise KeyError(f"Limb ({limb_type}, {side}) is not configured for this robot.")
        self._data[key] = value

    def get(self, limb_type, side):
        key = (limb_type, side)
        if key not in self._data:
            raise KeyError(f"Limb ({limb_type}, {side}) is not configured for this robot.")
        return self._data[key]

    def __getitem__(self, key):
        if isinstance(key, tuple):
            return self.get(*key)
        elif isinstance(key, ArmLegSide):
            limb_types = {limb for (limb, _) in self._data}
            if len(limb_types) == 1:
                only_limb = next(iter(limb_types))
                return self.get(only_limb, key)
            else:
                raise ValueError(f"Ambiguous key: multiple limb types present, cannot infer type for side {key}")
        else:
            raise ValueError(f"Key is not valid: expected (limb, side) tuple or ArmLegSide, got {key}")
        

    def __setitem__(self, key, value):
        """
            Sets the value for one or more limbs in the LimbData instance, based on the key.

            This method supports three types of keys:
            - A tuple of (ArmLeg, ArmLegSide): directly sets the value for that specific limb.
            - An ArmLegSide: sets the value for the limb with that side, but only if the instance has exactly one limb type.
            - An ArmLeg: sets the value for all limbs of that type.

            Args:
                key (tuple, ArmLegSide, or ArmLeg): The limb identifier(s).
                value (any): The value to set.

            Raises:
                ValueError: If the key is ambiguous or invalid, or if multiple limb types are present
                            when using a side-only key.
                TypeError: If force_same_value_type is True and value has the wrong type.
        """
        if isinstance(key, tuple):
            self.set(*key, value)

        elif isinstance(key, ArmLegSide):
            limb_types = {limb for (limb, _) in self._data}
            if len(limb_types) == 1:
                only_limb = next(iter(limb_types))
                self.set(only_limb, key, value)
            else:
                raise ValueError(f"Ambiguous key: multiple limb types present, cannot infer type for side {key}")
        elif isinstance(key, ArmLeg):
            self.set_all_of_type(key, value)

        else:
            raise ValueError(f"Key is not valid: expected (limb, side) tuple or ArmLegSide, got {key}")
        
    def set_all_of_type(self, limb_type: ArmLeg, value):
        """
            Updates all entries of the specified limb type to the given value.

            Args:
                limb_type (ArmLeg): The limb type to update (e.g., ArmLeg.ARM).
                value (any): The value to assign.

            Raises:
                TypeError: If force_same_value_type is True and value has the wrong type.
        """
        # Enforce type consistency if needed
        self.check_data(value)

        for (limb, side) in self._data:
            if limb == limb_type:
                self._data[(limb, side)] = value

        self._empty = False  # since we've populated some values

    def first_value_of_type(self, limb_type: ArmLeg):
        """
            Returns the first value for a given limb type, regardless of side.

            Args:
                limb_type (ArmLeg): The limb type (e.g., ArmLeg.ARM).

            Returns:
                The value associated with the first limb of the specified type.

            Raises:
                KeyError: If there is no limb of the specified type.
        """
        for (limb, _), value in self._data.items():
            if limb == limb_type:
                return value
        raise KeyError(f"No limb of type {limb_type} found.")


    def __contains__(self, key):
        return key in self._data
    
    def __iter__(self):
        return iter(self._data)

    def items(self):
        return self._data.items()

    def keys(self):
        return self._data.keys()

    def values(self):
        return self._data.values()
    
    def __len__(self):
        return len(self._data)
    
    def limb_count(self):
        return len(self)
    
    def arm_count(self):
        return self.count_limb_type(ArmLeg.ARM)

    def leg_count(self):
        return self.count_limb_type(ArmLeg.LEG)

    def wheel_count(self):
        return self.count_limb_type(ArmLeg.WHEEL)

    def count_limb_type(self, limb_type: ArmLeg):
        return sum(1 for limb, _ in self._data if limb == limb_type)
    
    def has_limb_type(self, limb_type: ArmLeg) -> bool:
        """
        Checks whether the specified limb type is present in the robot configuration.

        Args:
            limb_type (ArmLeg): The type of limb to check (e.g., ARM, LEG, WHEEL).

        Returns:
            bool: True if at least one limb of the given type is present, False otherwise.
        """
        return any(limb == limb_type for (limb, _) in self._data)
    
    def has_arm(self) -> bool:
        return self.has_limb_type(ArmLeg.ARM)

    def has_leg(self) -> bool:
        return self.has_limb_type(ArmLeg.LEG)

    def has_wheel(self) -> bool:
        return self.has_limb_type(ArmLeg.WHEEL)
    
    def has_only_limb_type(self, limb_type: ArmLeg) -> bool:
        """
        Checks whether all limbs in the configuration are of the specified type.

        Args:
            limb_type (ArmLeg): The limb type to check against.

        Returns:
            bool: True if all configured limbs are of the given type, False otherwise.
        """
        return all(limb == limb_type for (limb, _) in self._data)
    
    def has_only_arms(self) -> bool:
        return self.has_only_limb_type(ArmLeg.ARM)

    def has_only_legs(self) -> bool:
        return self.has_only_limb_type(ArmLeg.LEG)

    def has_only_wheels(self) -> bool:
        return self.has_only_limb_type(ArmLeg.WHEEL)
    
    @staticmethod
    def _predefined_config(name: str):
        name = name.strip().lower()        
        if   name in ["single arm", "one arm"]:
            config = [
                (ArmLeg.ARM, ArmLegSide.NONE)
            ]
        elif name in ["dual arm", "two arms"]:
            config = [
                (ArmLeg.ARM, ArmLegSide.LEFT),
                (ArmLeg.ARM, ArmLegSide.RIGHT)
            ]
        elif name == "dual arm wheeled":
            config = [
                (ArmLeg.ARM, ArmLegSide.LEFT),
                (ArmLeg.ARM, ArmLegSide.RIGHT),
                (ArmLeg.WHEEL, ArmLegSide.NONE)
            ]
        elif name in ["triple arm", "three arms"]:
            config = [
                (ArmLeg.ARM, ArmLegSide.LEFT),
                (ArmLeg.ARM, ArmLegSide.RIGHT),
                (ArmLeg.ARM, ArmLegSide.MIDDLE)
            ]
        elif name in ["quadruple arm", "four arms"]:
            config = [
                (ArmLeg.ARM, ArmLegSide.LEFT),
                (ArmLeg.ARM, ArmLegSide.RIGHT),
                (ArmLeg.ARM, ArmLegSide.LEFT2),
                (ArmLeg.ARM, ArmLegSide.RIGHT2)
            ]
        elif name == "humanoid":
            config = [
                (ArmLeg.ARM, ArmLegSide.LEFT),
                (ArmLeg.ARM, ArmLegSide.RIGHT),
                (ArmLeg.LEG, ArmLegSide.LEFT),
                (ArmLeg.LEG, ArmLegSide.RIGHT)
            ]        
        elif name in ["grievous", "four arms humanoid"]:
            config = [
                (ArmLeg.ARM, ArmLegSide.LEFT),
                (ArmLeg.ARM, ArmLegSide.RIGHT),
                (ArmLeg.ARM, ArmLegSide.LEFT2),
                (ArmLeg.ARM, ArmLegSide.RIGHT2),
                (ArmLeg.LEG, ArmLegSide.LEFT),
                (ArmLeg.LEG, ArmLegSide.RIGHT)
            ]
        elif name == "empty":
            config = [
            ]
        else:
            raise ValueError(f"Robot configuration '{name}' is unknown. Valid options are: "
                             "'humanoid', 'dual arm', 'dual arm wheeled', 'single arm', 'empty'.")
        return config
