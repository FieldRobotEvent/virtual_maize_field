from __future__ import annotations

import re
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

    def __repr__(self) -> str:
        return f"GazeboModel: {self.model_name}"


@dataclass
class GeneratedGazeboModel:
    model_regex: re.Pattern
    age_regex: re.Pattern = re.compile(".+_day_([0-9]+)")

    def get_models_by_age(self, days: list[int]) -> dict[str, GazeboModel]:
        models_folder = Path(__file__).parents[3] / "models"
        assert models_folder.is_dir(), "Cannot find virtual_maize_field model folder!"

        maize_models = {}

        for model_folder in models_folder.glob("**/"):
            if re.match(self.model_regex, model_folder.name):
                result = re.search(self.age_regex, model_folder.stem)
                model_days = result.groups()[0]

                if int(model_days) in map(int, days):
                    maize_models[model_folder.name] = GazeboModel(model_folder.stem)
        return maize_models


CROP_MODELS = {
    # "cylinder": GazeboModel("cylinder"),
    "maize_01": GazeboModel("maize_01"),
    "maize_02": GazeboModel("maize_02"),
    "generated_maize": GeneratedGazeboModel(re.compile("maize_[0-9]+_day_[0-9]+")),
}

WEED_MODELS = {
    "nettle": GazeboModel("nettle"),
    "unknown_weed": GazeboModel("unknown_weed"),
}

LITTER_MODELS = {
    "ale": GazeboModel("ale", default_roll=1.5707963267948966),
    "beer": GazeboModel("beer", default_roll=1.5707963267948966),
    "coke_can": GazeboModel("coke_can", default_roll=1.5707963267948966),
    "retro_pepsi_can": GazeboModel("retro_pepsi_can", default_roll=1.5707963267948966),
}

OBSTACLE_MODELS = {
    "box": GazeboModel("box"),
    "stone_01": GazeboModel("stone_01"),
    "stone_02": GazeboModel("stone_02"),
}

MARKER_MODELS = {
    "location_marker_a": GazeboModel(
        "location_marker_a", static=True, ghostable=False, random_yaw=False
    ),
    "location_marker_b": GazeboModel(
        "location_marker_b", static=True, ghostable=False, random_yaw=False
    ),
}

AVAILABLE_MODELS = {
    **CROP_MODELS,
    **WEED_MODELS,
    **LITTER_MODELS,
    **OBSTACLE_MODELS,
    **MARKER_MODELS,
}
