from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
from xml.etree import ElementTree


@dataclass
class GazeboModel:
    model_name: str
    default_x: float = 0.0
    default_y: float = 0.0
    default_z: float = 0.0
    default_roll: float = 0.0
    default_pitch: float = 0.0
    default_yaw: float = 0.0
    static: bool = False
    ghostable: bool = True
    random_yaw: bool = True

    __model_visual: str | None = None

    def get_model_visual(self) -> str:
        if self.__model_visual is None:
            models_folder = Path(__file__).parents[3] / "models"
            assert (
                models_folder.is_dir()
            ), "Cannot find virtual_maize_field model folder!"

            model_folder = models_folder / self.model_name
            assert model_folder.is_dir(), f"Cannot find model {self.model_name}!"

            root = ElementTree.parse(model_folder / "model.sdf").getroot()
            self.__model_visual = ElementTree.tostring(root.find(".//visual")).decode(
                "utf-8"
            )

        return self.__model_visual


AVAILABLE_CROP_TYPES = {
    # "cylinder": GazeboModel("cylinder"),
    "maize_01": GazeboModel("maize_01"),
    "maize_02": GazeboModel("maize_02"),
}

AVAILABLE_WEED_TYPES = {
    "nettle": GazeboModel("nettle"),
    "unknown_weed": GazeboModel("unknown_weed"),
}

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

AVAILABLE_MARKERS = {
    "location_marker_a": GazeboModel(
        "location_marker_a", static=True, ghostable=False, random_yaw=False
    ),
    "location_marker_b": GazeboModel(
        "location_marker_b", static=True, ghostable=False, random_yaw=False
    ),
}

AVAILABLE_MODELS = {
    **AVAILABLE_CROP_TYPES,
    **AVAILABLE_WEED_TYPES,
    **AVAILABLE_LITTER_TYPES,
    **AVAILABLE_OBSTACLES,
    **AVAILABLE_MARKERS,
}
