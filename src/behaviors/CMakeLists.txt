# Copyright (c) 2023 The ZMK Contributors
# SPDX-License-Identifier: MIT

target_sources_ifdef(CONFIG_ZMK_POINTER_2S_MIXER app PRIVATE behavior_p2sm_sens.c)
target_sources_ifdef(CONFIG_ZMK_POINTER_2S_MIXER app PRIVATE behavior_p2sm_twist_toggle.c)
