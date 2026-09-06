# Copyright (c) 2019 Intel Corporation
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import yaml
import tempfile
import launch


class RewrittenYaml(launch.Substitution):
    """Substitution that modifies the given YAML file."""

    def __init__(
        self,
        source_file,
        param_rewrites,
        root_key=None,
        key_rewrites=None,
        convert_types=False,
    ):
        super().__init__()
        from launch.utilities import normalize_to_list_of_substitutions
        self.__source_file = normalize_to_list_of_substitutions(source_file)
        self.__param_rewrites = {}
        self.__key_rewrites = {}
        self.__convert_types = convert_types
        self.__root_key = None
        for key in param_rewrites:
            self.__param_rewrites[key] = normalize_to_list_of_substitutions(
                param_rewrites[key]
            )
        if key_rewrites is not None:
            for key in key_rewrites:
                self.__key_rewrites[key] = normalize_to_list_of_substitutions(
                    key_rewrites[key]
                )
        if root_key is not None:
            self.__root_key = normalize_to_list_of_substitutions(root_key)

    @property
    def name(self):
        return self.__source_file

    def describe(self):
        return ""

    def perform(self, context):
        yaml_filename = launch.utilities.perform_substitutions(context, self.name)
        rewritten_yaml = tempfile.NamedTemporaryFile(mode="w", delete=False)
        param_rewrites, keys_rewrites = self.resolve_rewrites(context)
        data = yaml.safe_load(open(yaml_filename, "r"))
        if self.__root_key is not None:
            root_key = launch.utilities.perform_substitutions(context, self.__root_key)
            if root_key:
                data = {root_key: data}
        yaml.dump(data, rewritten_yaml)
        rewritten_yaml.close()
        return rewritten_yaml.name

    def resolve_rewrites(self, context):
        resolved_params = {}
        for key in self.__param_rewrites:
            resolved_params[key] = launch.utilities.perform_substitutions(
                context, self.__param_rewrites[key]
            )
        resolved_keys = {}
        for key in self.__key_rewrites:
            resolved_keys[key] = launch.utilities.perform_substitutions(
                context, self.__key_rewrites[key]
            )
        return resolved_params, resolved_keys