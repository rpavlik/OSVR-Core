set(tests JointClientKit)
if(BUILD_SERVER_EXAMPLES) # need the AnalogSync and example
    list(APPEND tests JointClientKitWithInterface JointClientKitWithAsyncInterface)
endif()
foreach(test ${tests})
    add_executable(Test${test}
        ${test}.cpp)
    target_link_libraries(Test${test} osvrClientKitCpp osvrJointClientKit osvr_cxx11_flags)
    osvr_setup_gtest(Test${test})
endforeach()
