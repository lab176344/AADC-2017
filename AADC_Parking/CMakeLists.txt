# add the include directory of the filter
include_directories(../include)
# declare our filter
set(FILTER_NAME user_Parking)

adtf_add_filter(${FILTER_NAME}
    stdafx.h
    cParking.h
    cParking.cpp
)

adtf_set_folder(${FILTER_NAME} Parking) 

# Specify where it should be installed to
adtf_install_plugin(${FILTER_NAME} ${CMAKE_INSTALL_BINARY})
