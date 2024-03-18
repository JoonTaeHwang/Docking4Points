from dataclasses import dataclass,field
import numpy as np

@dataclass
class LiDAR:
    pcd: np.ndarray
    reflectivity: np.ndarray
    voxel2d: np.ndarray = field(default_factory=lambda:np.empty(0))
    voxel2d_prev: np.ndarray = field(default_factory=lambda:np.empty(0))
    running:bool = True
    initial_data: bool = False
    metadata = None
    source = None

@dataclass
class DK:
    dkpoints: np.ndarray
    center_point: np.ndarray

@dataclass
class SETTINGS:
    boatwidth: float
    boatlenth: float
    voxel_size:float
    metadata_path:str
    pcap_path:str
    




