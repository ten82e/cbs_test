# Copyright (c) 2024 Your Org <your.email@example.com> // Placeholder
# Based on nsim CMakeLists.txt, Copyright (c) Cisco and/or its affiliates.
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at:
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

# Change plugin name from nsim to cbs
add_vpp_plugin(cbs
  SOURCES
  cbs.c          # Renamed from nsim.c
  node.c         # Contains cbs-cross-connect and cbs-output-feature nodes
  cbs_input.c    # Renamed from nsim_input.c

  MULTIARCH_SOURCES
  cbs_input.c
  node.c

  API_FILES
  cbs.api        # Renamed from nsim.api

  API_TEST_SOURCES
  cbs_test.c     # Renamed from nsim_test.c
)