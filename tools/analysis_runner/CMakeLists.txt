# spade_analysis_runner (W2c; INT-7): the headless process a queued analysis
# run runs in. A host launches it on a run folder with
# spade::analysis::launch_run(), by its explicit path.
#
# Outside engine/, so it may read the steady clock (M1B keeps wall-clock
# symbols out of engine/ and is not widened for it). It links installed
# spade:: targets and includes installed headers only, as a consumer would.
# Installed into bin/ and exported as spade::analysis_runner, so a consumer
# finds it as $<TARGET_FILE:spade::analysis_runner>.
add_executable(spade_analysis_runner main.cpp)
add_executable(spade::analysis_runner ALIAS spade_analysis_runner)
set_target_properties(spade_analysis_runner PROPERTIES EXPORT_NAME analysis_runner)
target_link_libraries(spade_analysis_runner PRIVATE spade::analysis spade_warnings)
target_compile_features(spade_analysis_runner PRIVATE cxx_std_23)

install(TARGETS spade_analysis_runner
    EXPORT spadeTargets
    RUNTIME DESTINATION bin
)
