from dataclasses import dataclass


@dataclass
class GazeboModel:
    name: str
    default_roll: float = 0.0
    default_pitch: float = 0.0
    default_yaw: float = 0.0
    static: bool = False


AVAILABLE_CROP_TYPES = {
    "cylinder": GazeboModel("cylinder"),
    "maize_01": GazeboModel("maize_01"),
    "maize_02": GazeboModel("maize_02"),
}
AVAILABLE_WEED_TYPES = {"nettle": GazeboModel("nettle"), "unknown_weed": GazeboModel("unknown_weed")}
AVAILABLE_LITTER_TYPES = {
    "ale": GazeboModel("ale", default_roll=1.5707963267948966),
    "beer": GazeboModel("beer", default_roll=1.5707963267948966),
    "coke_can": GazeboModel("coke_can", default_roll=1.5707963267948966),
    "retro_pepsi_can": GazeboModel("retro_pepsi_can", default_roll=1.5707963267948966),
}
AVAILABLE_OBSTACLES = {
    "box": GazeboModel("box"),
    "stone_01": GazeboModel("stone_01"),
    "stone_02": GazeboModel("stone_02"),
}