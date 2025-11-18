add_executable(ik_benchmarks
    ik_benchmarks.cpp
)

target_compile_features(ik_benchmarks PRIVATE cxx_std_20)

target_link_libraries(ik_benchmarks
    PRIVATE
        urdfx
        benchmark::benchmark
        benchmark::benchmark_main
        spdlog::spdlog
)

target_compile_definitions(ik_benchmarks
    PRIVATE
    URDFX_UR5_URDF_PATH="${PROJECT_SOURCE_DIR}/ur5_urdf/ur5e.urdf"
)
