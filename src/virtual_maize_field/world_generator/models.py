from __future__ import annotations

import re
from dataclasses import dataclass, fields
from pathlib import Path
from xml.etree import ElementTree

VIRTUAL_MAIZE_FIELD_MODELS_FOLDER = models_folder = Path(__file__).parents[3] / "models"


@dataclass
class GazeboModel:
    model_name: str = "maize_01"
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
            model_folder = VIRTUAL_MAIZE_FIELD_MODELS_FOLDER / self.model_name
            assert model_folder.is_dir(), f"Cannot find model {self.model_name}!"

            root = ElementTree.parse(model_folder / "model.sdf").getroot()

            self.__model_visual = ""
            for visual in root.findall(".//visual"):
                self.__model_visual += ElementTree.tostring(visual).decode("utf-8")

        return self.__model_visual

    def __repr__(self) -> str:
        return f"GazeboModel: {self.model_name}"


@dataclass
class GazeboModelsFromRegex(GazeboModel):
    """
    Dataclass that gathers models by a regular expression.

    :param model_name_regex: Regex to match a series of model folder names. This regex should have
                             a single capturing group matching the model name.
    """

    model_name_regex: re.Pattern = re.compile(r"(maize_[0-9]+)_.+")

    __models: list[GazeboModel] | None = None

    def __len__(self) -> int:
        if self.__models is None:
            self._gather_models()

        return len(self.__models)

    @property
    def list(self) -> list[GazeboModel]:
        if self.__models is None:
            self._gather_models()

        return self.__models

    def _gather_models(self) -> None:
        self.__models = []

        for model_folder in VIRTUAL_MAIZE_FIELD_MODELS_FOLDER.glob("**/"):
            name_match = re.search(self.model_name_regex, model_folder.stem)

            if name_match is not None:
                model_name = name_match.groups()[0]

                # Get the parameters belonging to the GazeboModel class
                gazebo_model_parameters = {}
                gazebo_model_fields = [
                    f.name for f in fields(GazeboModel) if not f.name.startswith("_")
                ]
                for field in gazebo_model_fields:
                    gazebo_model_parameters[field] = getattr(self, field)

                # Set the model name
                gazebo_model_parameters["model_name"] = model_name

                self.__models.append(GazeboModel(**gazebo_model_parameters))

        if len(self.__models) == 0:
            print(
                "\033[91mError: Cannot find any model matching the regular expression "
                f"'{self.model_name_regex.pattern}' in the models folder! Did you add "
                "the models to that folder?\033[0m"
            )
            exit(1)

    def __repr__(self) -> str:
        return f"GazeboModelsFromRegex: {self.model_name_regex.pattern} (matched {len(self)} instances)"


def to_gazebo_models(
    models: dict[str, GazeboModel | GazeboModelsFromRegex],
    model_names: list[str],
) -> dict[str, GazeboModel]:
    """
    Converts dict of different types of Gazebo models to dict with only Gazebo models.
    """
    output_dict = {}

    for model_name, model in models.items():
        if model_name in model_names:
            if isinstance(model, GazeboModelsFromRegex):
                m_dict = {m.model_name: m for m in model.list}
                output_dict.update(to_gazebo_models(m_dict, list(m_dict.keys())))

            elif isinstance(model, GazeboModel):
                output_dict[model.model_name] = model

            else:
                print(f"ERROR: unknown instance {type(model)}!")

    return output_dict


CROP_MODELS = {
    # "cylinder": GazeboModel("cylinder"),
    "maize_01": GazeboModel(model_name="maize_01"),
    "maize_02": GazeboModel(model_name="maize_02"),
}

WEED_MODELS = {
    "nettle": GazeboModel(model_name="nettle"),
    "unknown_weed": GazeboModel(model_name="unknown_weed"),
    "dandelion": GazeboModelsFromRegex(
        model_name_regex=re.compile(r"(dandelion_[0-9]+)")
    ),
}

LITTER_MODELS = {
    "ale": GazeboModel(model_name="ale", default_roll=1.5707963267948966),
    "beer": GazeboModel(model_name="beer", default_roll=1.5707963267948966),
    "coke_can": GazeboModel(
        model_name="coke_can", default_roll=1.5707963267948966, default_z=0.025
    ),
    "retro_pepsi_can": GazeboModel(
        model_name="retro_pepsi_can",
        default_roll=1.5707963267948966,
        default_z=0.025,
    ),
}

OBSTACLE_MODELS = {
    "box": GazeboModel(model_name="box"),
    "stone_01": GazeboModel(model_name="stone_01"),
    "stone_02": GazeboModel(model_name="stone_02"),
}

MARKER_MODELS = {
    "location_marker_a": GazeboModel(
        model_name="location_marker_a", static=True, ghostable=False, random_yaw=False
    ),
    "location_marker_b": GazeboModel(
        model_name="location_marker_b", static=True, ghostable=False, random_yaw=False
    ),
}

AVAILABLE_MODELS = {
    **CROP_MODELS,
    **WEED_MODELS,
    **LITTER_MODELS,
    **OBSTACLE_MODELS,
    **MARKER_MODELS,
}
