from __future__ import annotations

import re
from dataclasses import dataclass, fields
from pathlib import Path
from xml.etree import ElementTree

from cv2 import dft
from regex import P

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
class GazeboGrowthModel(GazeboModel):
    """
    Dataclass containing growth models. This dataclass contains multiple GazeboModels
    at different stages in growth.

    :param model_name_regex: Regex to match all model folders of a single model with
                             different growth days.
    :param age_regex: Regex to match the growth day within the model name. This regex should
                      have one capturing group matching the day number.
    """

    model_name_regex: re.Pattern = re.compile(r"maize_001_day_[0-9]+")
    age_regex: re.Pattern = re.compile(r".+_day_([0-9]+)")

    __models_by_day: dict[int, GazeboModel] | None = None

    def __getitem__(self, day_number: int) -> GazeboModel | None:
        if self.__models_by_day is None:
            self._gather_models()

        if day_number not in self.available_days:
            print(f"WARNING: could not find model day {day_number}!")
            return None

        return self.__models_by_day[day_number]

    def __len__(self) -> int:
        if self.__models_by_day is None:
            self._gather_models()

        return len(self.__models_by_day)

    @property
    def available_days(self) -> list[int]:
        if self.__models_by_day is None:
            self._gather_models()

        return list(self.__models_by_day.keys())

    def _gather_models(self) -> None:
        self.__models_by_day = {}

        for model_folder in VIRTUAL_MAIZE_FIELD_MODELS_FOLDER.glob("**/"):
            if re.match(self.model_name_regex, model_folder.stem):
                model_days_match = re.search(self.age_regex, model_folder.stem)
                if model_days_match is not None:
                    model_days = int(model_days_match.groups()[0])

                    # Get the parameters belonging to the GazeboModel class
                    gazebo_model_parameters = {}
                    gazebo_model_fields = [
                        f.name
                        for f in fields(GazeboModel)
                        if not f.name.startswith("_")
                    ]
                    for field in gazebo_model_fields:
                        gazebo_model_parameters[field] = getattr(self, field)

                    # Set the model name
                    gazebo_model_parameters["model_name"] = model_folder.name

                    self.__models_by_day[model_days] = GazeboModel(
                        **gazebo_model_parameters
                    )

    def __repr__(self) -> str:
        return f"GazeboGrowthModel: {self.model_name_regex.pattern} (ages: {[a for a in sorted(self.available_days)]})"


@dataclass
class GazeboModelsFromRegex(GazeboModel):
    """
    Dataclass that gathers models by a regular expression. If age_regex is defined, GazeboGrowthModels are
    returned instead of GazeboModels.

    :param model_name_regex: Regex to match a series of model folder names. This regex should have
                             a single capturing group matching the model name.
    :param age_regex: Regex to match the age in the model name if the model is a GazeboGrowthModel. Otherwise
                      use None.
    """

    model_name_regex: re.Pattern = re.compile(r"(maize_[0-9]+)_.+")
    age_regex: re.Pattern | None = None

    __models: list[GazeboModel | GazeboGrowthModel] | None = None

    def __len__(self) -> int:
        if self.__models is None:
            self._gather_models()

        return len(self.__models)

    @property
    def list(self) -> list[GazeboModel | GazeboGrowthModel]:
        if self.__models is None:
            self._gather_models()

        return self.__models

    def _gather_models(self) -> None:
        self.__models = []
        added_names = []

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
                gazebo_model_parameters["model_name"] = model_folder.name

                if self.age_regex is None:
                    self.__models.append(GazeboModel(**gazebo_model_parameters))
                elif model_name not in added_names:
                    new_regex = re.sub(
                        r"\([^)]*\)", model_name, self.model_name_regex.pattern
                    )

                    self.__models.append(
                        GazeboGrowthModel(
                            model_name_regex=re.compile(new_regex),
                            age_regex=self.age_regex,
                            **gazebo_model_parameters,
                        )
                    )
                    added_names.append(model_name)

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
    models: dict[str, GazeboModel | GazeboGrowthModel | GazeboModelsFromRegex],
    model_names: list[str],
    ages: list[int] | None = None,
) -> dict[str, GazeboModel]:
    """
    Converts dict of different types of Gazebo models to dict with only Gazebo models.
    """
    output_dict = {}

    for model_name, model in models.items():
        if model_name in model_names:
            if isinstance(model, GazeboModelsFromRegex):
                m_dict = {m.model_name: m for m in model.list}
                output_dict.update(to_gazebo_models(m_dict, list(m_dict.keys()), ages))

            elif isinstance(model, GazeboGrowthModel) and ages is not None:
                for age in ages:
                    model_at_age = model[age]
                    if model_at_age is not None:
                        output_dict[model_at_age.model_name] = model_at_age

            elif isinstance(model, GazeboModel):
                output_dict[model.model_name] = model

            else:
                print(f"ERROR: unknown instance {type(model)} with ages {ages}!")

    return output_dict


CROP_MODELS = {
    # "cylinder": GazeboModel("cylinder"),
    "maize_01": GazeboModel(model_name="maize_01"),
    "maize_02": GazeboModel(model_name="maize_02"),
    "generated_maize": GazeboModelsFromRegex(
        model_name_regex=re.compile(r"(maize_[0-9]+)_day_[0-9]+"),
        age_regex=re.compile(r".+_day_([0-9]+)"),
    ),
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
    "coke_can": GazeboModel(model_name="coke_can", default_roll=1.5707963267948966),
    "retro_pepsi_can": GazeboModel(
        model_name="retro_pepsi_can", default_roll=1.5707963267948966
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
