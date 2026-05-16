find_package(Eigen3 3.3 REQUIRED NO_MODULE)
include_directories(${EIGEN3_INCLUDE_DIRS})

add_executable(distry_mlat26_cl
    main.cpp
    UwbSecurityApp.cpp
    UwbHeader.cpp
    Trajectories.cpp
    EKF.cpp
    UWBChannel.cpp
)

target_link_libraries(distry_mlat26_cl
    ${libcore}
    ${libnetwork}
    ${libmobility}
    ${libinternet}
    ${libapplications}
    ${libwifi}
    ${libspectrum}
    ${libpropagation}
    Eigen3::Eigen
)
