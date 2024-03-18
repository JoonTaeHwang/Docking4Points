
import numpy as np
from DKPoints import SETTINGS
from DKPoints.client import Builder
if __name__ == '__main__':
    

    settings = SETTINGS(
        boatwidth = 5.5, # 선석 폭
        boatlenth = 8,   # 선석 길이
        voxel_size = 0.1,
        metadata_path = 'log/WS_DK.json',
        pcap_path= 'log/WS_DK.pcap'
    )
    
    Calibration_builder = (
        Builder()
        .settings(settings)
        .build()
    )