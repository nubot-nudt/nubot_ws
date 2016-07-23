# Disabled until it's mature
#rosbuild_add_executable(find_camera find_camera.cpp)

add_executable(write_memory write_memory.cpp)
target_link_libraries(write_memory prosilica)

add_executable(read_memory read_memory.cpp)
target_link_libraries(read_memory prosilica)

add_executable(set_ip set_ip.cpp)
target_link_libraries(set_ip prosilica)

add_executable(set_inhibition set_inhibition.cpp)
target_link_libraries(set_inhibition prosilica)
