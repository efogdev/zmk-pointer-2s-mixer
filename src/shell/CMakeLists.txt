target_sources_ifdef(CONFIG_ZMK_P2SM_SHELL app PRIVATE p2sm_shell.c)
set_source_files_properties(p2sm_shell.c PROPERTIES COMPILE_FLAGS "-Os")
