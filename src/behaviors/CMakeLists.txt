# Copyright (c) 2023 The ZMK Contributors
# SPDX-License-Identifier: MIT

target_sources_ifdef(CONFIG_ZMK_POINTER_2S_MIXER app PRIVATE behavior_p2sm_sens.c
                                                                behavior_p2sm_accel_toggle.c
                                                                behavior_p2sm_accel_adj.c)
