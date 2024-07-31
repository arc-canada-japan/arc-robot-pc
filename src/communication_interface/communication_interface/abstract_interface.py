from abc import ABC, abstractmethod
import os
from ament_index_python.packages import get_package_share_directory

CURRENT_PACKAGE = os.path.basename(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

class AbstractInterface(ABC):
    INTERFACE_NAME: str
    _subscriber_list: dict
    _publisher_list: dict
    _parameters: dict
    _config_file: str

    @property
    def INTERFACE_NAME(self) -> str:
        return self._interface_name
    
    @INTERFACE_NAME.setter
    def INTERFACE_NAME(self, interface_name: str) -> None:        
        self._interface_name = interface_name

    
    # -- Constructor --
    def __init__(self, interface_name: str):
        self.INTERFACE_NAME = interface_name
        self._subscriber_list = {}
        self._publisher_list = {}

        self._config_file = os.path.join(
        get_package_share_directory(CURRENT_PACKAGE),
        'config',
        self.INTERFACE_NAME.lower() + '.yaml'
        )

        self.load_parameters()

    @abstractmethod
    def load_parameters(self) -> None:
        pass

    @abstractmethod
    def connection_to_host(self, host: str, port: int) -> None:
        pass

    @abstractmethod
    def define_subscribers(self, sub_list: dict) -> None:
        pass

    @abstractmethod
    def define_publishers(self, pub_list: dict) -> None:
        pass

    @abstractmethod
    def publish(self, pub_topic: str, data) -> None:
        pass