find_package(Eigen3 3.3 REQUIRED NO_MODULE)
include_directories(${EIGEN3_INCLUDE_DIRS})
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include)


add_executable(distry_mlat26v4-nomanager
    src/main.cpp
    src/UwbSecurityApp.cpp
    src/UwbHeader.cpp
    src/Trajectories.cpp
    src/EKF.cpp
    src/UWBChannel.cpp
)

target_link_libraries(distry_mlat26v4-nomanager
    ${libcore}
    ${libnetwork}
    ${libmobility}
    ${libinternet}
    ${libapplications}
    ${libwifi}
    ${libspectrum}
    ${libpropagation}
    ${libnetanim}
    Eigen3::Eigen
)
