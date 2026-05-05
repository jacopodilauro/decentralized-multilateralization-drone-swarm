# CMakeLists.txt per simulazione TDOA UWB SwarmRaft in ns-3

# 1. Cerca Eigen (necessario per calcoli matriciali EKF)
find_package(Eigen3 3.3 REQUIRED NO_MODULE)
include_directories(${EIGEN3_INCLUDE_DIRS})

# 2. Definisci l'eseguibile
# Aggiungiamo i nuovi file (main.cpp, UwbSecurityApp.cpp, UwbHeader.cpp, ecc.)
add_executable(distry_mlat26
    main.cpp
    UwbSecurityApp.cpp
    UwbHeader.cpp
    Trajectories.cpp
    EKF.cpp
    UWBChannel.cpp
)

# 3. Collega le librerie necessarie di NS-3 e di Eigen
target_link_libraries(distry_mlat26
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
