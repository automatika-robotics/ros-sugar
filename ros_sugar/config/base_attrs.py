import json
from typing import Any, Callable, Dict, Optional, Union, get_args
from copy import deepcopy
import numpy as np
from attrs import asdict, define, fields_dict
from attrs import has as attrs_has
from omegaconf import OmegaConf


@define
class BaseAttrs:
    """
    Implements setattr method to re-use validators at set time
    """

    def __setattr__(self, name: str, value: Any) -> None:
        """Call the validator when we set the field (by default it only runs on __init__)"""
        for attribute in [
            a for a in getattr(self.__class__, "__attrs_attrs__", []) if a.name == name
        ]:
            if attribute.validator is not None:
                attribute.validator(self, attribute, value)
        super().__setattr__(name, value)

    def __str__(self) -> str:
        """
        Pretty print of class attributes/values

        :return: _description_
        :rtype: str
        """
        print_statement = "\n"

        first_level_keys = [attr.name for attr in self.__attrs_attrs__]
        first_level_values = [getattr(self, key) for key in first_level_keys]

        for name, value in zip(first_level_keys, first_level_values):
            print_statement += f"{name}: {value}\n"

        return print_statement

    @classmethod
    def __is_union_type(cls, some_type) -> bool:
        """
        Helper method to check if a type is from typing.Union

        :param some_type: Some type to check
        :type some_type: type

        :return: If type is from typing.Union
        :rtype: bool
        """
        return getattr(some_type, "__origin__", None) is Union

    @classmethod
    def __isinstance_of_union(cls, obj, union_type) -> bool:
        """
        Helper method to check if a type is from typing.Union

        :param obj: _description_
        :type obj: _type_
        :param union_type: _description_
        :type union_type: _type_
        :return: _description_
        :rtype: _type_
        """
        types = get_args(union_type)
        return any(isinstance(obj, t) for t in types)

    def asdict(self, filter: Optional[Callable] = None) -> dict:
        """Convert class to dict.
        :rtype: dict
        """
        return asdict(self, filter=filter)

    def __check_value_against_attr_type(
        self, key: str, value: Any, attribute_to_set: Any, attribute_type: type
    ):
        """Helper method to check if a given value can be parsed to a class attribute

        :param key: Attribute key name
        :type key: str
        :param value: New value to set
        :type value: Any
        :param attribute_to_set: Class attribute to set
        :type attribute_to_set: Any
        :param attribute_type: Class attribute type
        :type attribute_type: type

        :raises TypeError: If value is of incompatible type

        :return: Updated value
        :rtype: Any
        """
        # Union typing requires special treatment
        if self.__is_union_type(attribute_type):
            if not self.__isinstance_of_union(value, attribute_type):
                raise TypeError(
                    f"Trying to set with incompatible type. Attribute {key} expecting '{type(attribute_to_set)}' got '{type(value)}'"
                )
        elif isinstance(value, list) and attribute_type is np.ndarray:
            # Turn list into numpy array
            value = np.array(value)

        # If not a Union type -> check using isinstance
        if not isinstance(value, attribute_type):
            raise TypeError(
                f"Trying to set with incompatible type. Attribute {key} expecting '{type(attribute_to_set)}' got '{type(value)}'"
            )
        return value

    def __parse_from_serialized_list(self, list_attr: list, value: list) -> list:
        """Helper method to parse attribute value from a list of serialized values

        :param list_attr: _description_
        :type list_attr: list
        :param value: _description_
        :type value: list
        :raises TypeError: _description_
        :return: _description_
        :rtype: list
        """
        new_list = []
        attr_val = list_attr[0]
        for val in value:
            if hasattr(attr_val, "__attrs_attrs__"):
                if not isinstance(val, Dict):
                    raise TypeError(
                        f"Trying to set with incompatible type. Attribute expecting list of dictionaries got list of '{type(val)}'"
                    )
                attr_val.from_dict(val)
            new_list.append(deepcopy(attr_val))
        return new_list

    def from_dict(self, dict_obj: Dict) -> None:
        """
        Gets attributes values from given dictionary

        :param dict_obj: Dictionary {attribute_name: attribute_value}
        :type dict_obj: Dict

        :raises ValueError: If attribute_name in dictionary does not exists in class attributes
        :raises TypeError: If attribute_value type in dictionary does not correspond to class attribute type
        """
        for key, value in dict_obj.items():
            if key not in asdict(self).keys():
                continue
            attribute_to_set = getattr(self, key)
            attribute_type = fields_dict(self.__class__)[key].type
            # Check for nested classes
            if hasattr(attribute_to_set, "__attrs_attrs__"):
                if not isinstance(value, Dict):
                    raise TypeError(
                        f"Trying to set with incompatible type. Attribute {key} expecting dictionary got '{type(value)}'"
                    )
                attribute_to_set.from_dict(value)
            elif isinstance(attribute_to_set, list):
                setattr(
                    self,
                    key,
                    self.__parse_from_serialized_list(attribute_to_set, value),
                )
            else:
                # Handle Any typing as it cannot be checked with isinstance
                if attribute_type is Any:
                    continue
                elif attribute_type:
                    value = self.__check_value_against_attr_type(
                        key, value, attribute_to_set, attribute_type
                    )
                setattr(self, key, value)

    def from_yaml(
        self,
        file_path: str,
        nested_root_name: str | None = None,
        get_common: bool = False,
    ) -> None:
        """
        Update class attributes from yaml

        :param file_path: Path to config file (.yaml)
        :type file_path: str
        :param nested_root_name: Nested root name for the config, defaults to None
        :type nested_root_name: str | None, optional
        """
        # Load the YAML file
        raw_config = OmegaConf.load(file_path)

        # check for root name if given
        if nested_root_name:
            config = OmegaConf.select(raw_config, nested_root_name)
            if get_common:
                extra_config = OmegaConf.select(raw_config, "/**")
            else:
                extra_config = None
        else:
            config = raw_config
            extra_config = None

        for attr in self.__attrs_attrs__:
            # Check in config
            if hasattr(config, attr.name):
                attr_value = getattr(self, attr.name)
                # Check to handle nested config
                if attrs_has(attr.type):
                    root_name = f"{nested_root_name}.{attr.name}"

                    attr_value.from_yaml(file_path, root_name)

                    setattr(self, attr.name, attr_value)
                else:
                    setattr(self, attr.name, getattr(config, attr.name))

            # Check in the common config if present
            elif extra_config:
                if hasattr(extra_config, attr.name):
                    attr_value = getattr(self, attr.name)
                    # Check to handle nested config
                    if attrs_has(attr.type):
                        root_name = f"/**.{attr.name}"

                        attr_value.from_yaml(file_path, root_name)

                        setattr(self, attr.name, attr_value)
                    else:
                        setattr(self, attr.name, getattr(extra_config, attr.name))

    def to_json(self) -> Union[str, bytes, bytearray]:
        """
        Dump to json

        :return: _description_
        :rtype: str | bytes | bytearray
        """
        dictionary = asdict(self)
        serialized_dict = self.__dict_to_serialized_dict(dictionary)
        return json.dumps(serialized_dict)

    def __list_to_serialized_list(self, list_items: list) -> list:
        """Serialize List object items

        :param list_items: _description_
        :type list_items: list
        :return: _description_
        :rtype: list
        """
        serialized_list = []
        for item in list_items:
            if isinstance(item, BaseAttrs):
                serialized_list.append(item.to_json())
            elif isinstance(item, list):
                serialized_list.append(self.__list_to_serialized_list(item))
            elif isinstance(item, Dict):
                serialized_list.append(self.__dict_to_serialized_dict(item))
            elif type(item) not in [float, int, str, bool]:
                serialized_list.append(str(item))
            else:
                serialized_list.append(item)
        return serialized_list

    def __dict_to_serialized_dict(self, dictionary):
        """Serialize Dictionary object items

        :param dictionary: _description_
        :type dictionary: _type_
        :return: _description_
        :rtype: _type_
        """
        for name, value in dictionary.items():
            # Convert numpy array to list
            if isinstance(value, np.ndarray):
                value = value.tolist()

            if isinstance(value, list):
                dictionary[name] = self.__list_to_serialized_list(value)
            elif isinstance(value, Dict):
                dictionary[name] = self.__dict_to_serialized_dict(value)
            elif type(value) not in [float, int, str, bool]:
                dictionary[name] = str(value)
        return dictionary

    def from_json(self, json_obj: Union[str, bytes, bytearray]) -> None:
        """
        Gets attributes values from given json

        :param json_obj: Json object
        :type json_obj: str | bytes | bytearray
        """
        dict_obj = json.loads(json_obj)
        self.from_dict(dict_obj)

    def has_attribute(self, attr_name: str) -> bool:
        """
        Checks if class object has attribute with given name

        :param attr_name: _description_
        :type attr_name: str

        :return: If object has attribute with given name
        :rtype: bool
        """
        # Get nested attributes if there
        nested_names = attr_name.split(".")
        obj_to_set = self
        for name in nested_names:
            # Raise an error if the name does not exist in the class
            if not hasattr(obj_to_set, name):
                return False
            obj_to_set = getattr(obj_to_set, name)
        return True

    def get_attribute_type(self, attr_name: str) -> Optional[type]:
        """
        Gets type of given attribute name

        :param attr_name: _description_
        :type attr_name: str

        :raises AttributeError: If class does not have attribute with given name

        :return: Attribute type
        :rtype: type
        """
        # Get nested attributes if there
        nested_names = attr_name.split(".")
        name_to_set = nested_names[0]
        obj_to_set = self
        obj_class = self
        for name_to_set in nested_names:
            # Raise an error if the name does not exist in the class
            if not hasattr(obj_to_set, name_to_set):
                raise AttributeError(
                    f"Class '{self.__class__.__name__}' does not have an attribute '{attr_name}'"
                )
            obj_class = obj_to_set
            obj_to_set = getattr(obj_to_set, name_to_set)

        return fields_dict(obj_class.__class__)[name_to_set].type

    def update_value(self, attr_name: str, attr_value: Any) -> bool:
        """
        Updates the value of an attribute in the class

        :param attr_name: Attribute name - can be nested name
        :type attr_name: str
        :param attr_value: Attribute value
        :type attr_value: Any

        :raises AttributeError: If class does not containe attribute with given name
        :raises TypeError: If class attribute with given name if of different type

        :return: If attribute value is updated
        :rtype: bool
        """
        # Get nested attributes if there
        nested_names = attr_name.split(".")
        name_to_set = nested_names[0]
        obj_to_set = self
        obj_class = self
        for name_to_set in nested_names:
            # Raise an error if the name does not exist in the class
            if not hasattr(obj_to_set, name_to_set):
                raise AttributeError(
                    f"Class '{self.__class__.__name__}' does not have an attribute '{attr_name}'"
                )
            obj_class = obj_to_set
            obj_to_set = getattr(obj_to_set, name_to_set)

        attribute_type = fields_dict(obj_class.__class__)[name_to_set].type

        if not attribute_type:
            raise TypeError(
                f"Class '{self.__class__.__name__}' attribute '{attr_name}' type unknown"
            )

        if not isinstance(attr_value, attribute_type):
            raise TypeError(
                f"Class '{self.__class__.__name__}' attribute '{attr_name}' expecting type '{attribute_type}', got {type(attr_value)}"
            )
        setattr(obj_class, name_to_set, attr_value)
        return True
