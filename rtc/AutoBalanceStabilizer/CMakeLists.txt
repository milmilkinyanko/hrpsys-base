if("${CMAKE_CXX_COMPILER_ID}" STREQUAL "GNU" AND CMAKE_CXX_COMPILER_VERSION VERSION_LESS 5.1)
  message(WARNING "[rtc/AutoBalanceStabilizer] Your gcc version is less than 5.1. Skip Compiling AutoBalanceStabilizer")
  return()
endif()

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++14")

set(comp_sources AutoBalanceStabilizer.cpp AutoBalanceStabilizerService_impl.cpp ../ImpedanceController/JointPathEx.cpp ../ImpedanceController/RatsMatrix.cpp ../SequencePlayer/interpolator.cpp PreviewController.cpp GaitGenerator.cpp SimpleFullbodyInverseKinematicsSolver.h ../TorqueFilter/IIRFilter.cpp)
set(libs hrpModel-3.1 hrpCollision-3.1 hrpUtil-3.1 hrpsysBaseStub)

include_directories(${PROJECT_SOURCE_DIR}/rtc/SequencePlayer)

add_library(AutoBalanceStabilizer SHARED ${comp_sources})
target_link_libraries(AutoBalanceStabilizer PRIVATE Stabilizer ${libs})
set_target_properties(AutoBalanceStabilizer PROPERTIES PREFIX "")

add_library(Stabilizer SHARED Integrator.cpp TwoDofController.cpp Stabilizer.cpp ../ImpedanceController/JointPathEx.cpp ../ImpedanceController/RatsMatrix.cpp ../TorqueFilter/IIRFilter.h)
target_link_libraries(Stabilizer PRIVATE ${libs})

add_executable(AutoBalanceStabilizerComp AutoBalanceStabilizerComp.cpp ${comp_sources})
target_link_libraries(AutoBalanceStabilizerComp PRIVATE AutoBalanceStabilizer Stabilizer ${libs})

add_executable(testPreviewControllerABS testPreviewControllerABS.cpp ../ImpedanceController/RatsMatrix.cpp PreviewController.cpp)
target_link_libraries(testPreviewControllerABS ${libs})

add_executable(testGaitGeneratorABS testGaitGeneratorABS.cpp ../ImpedanceController/RatsMatrix.cpp PreviewController.cpp GaitGenerator.cpp ../SequencePlayer/interpolator.cpp)
target_link_libraries(testGaitGeneratorABS ${libs})
set_target_properties(testGaitGeneratorABS PROPERTIES COMPILE_DEFINITIONS "FOR_TESTGAITGENERATOR=1")

add_executable(testTwoDofControllerABS testTwoDofControllerABS.cpp TwoDofController.cpp Integrator.cpp)
target_link_libraries(testTwoDofControllerABS ${libs})

add_executable(testZMPDistributorABS testZMPDistributorABS.cpp ZMPDistributor.h ../ImpedanceController/JointPathEx.cpp)
target_link_libraries(testZMPDistributorABS ${libs})

add_test(testPreviewControllerNoGP testPreviewControllerABS --use-gnuplot false)
add_test(testGaitGeneratorTest0  testGaitGeneratorABS --test0 --use-gnuplot false)
add_test(testGaitGeneratorTest1  testGaitGeneratorABS --test1 --use-gnuplot false)
add_test(testGaitGeneratorTest2  testGaitGeneratorABS --test2 --use-gnuplot false)
add_test(testGaitGeneratorTest3  testGaitGeneratorABS --test3 --use-gnuplot false)
add_test(testGaitGeneratorTest4  testGaitGeneratorABS --test4 --use-gnuplot false)
add_test(testGaitGeneratorTest5  testGaitGeneratorABS --test5 --use-gnuplot false)
add_test(testGaitGeneratorTest6  testGaitGeneratorABS --test6 --use-gnuplot false)
add_test(testGaitGeneratorTest7  testGaitGeneratorABS --test7 --use-gnuplot false)
add_test(testGaitGeneratorTest8  testGaitGeneratorABS --test8 --use-gnuplot false)
add_test(testGaitGeneratorTest9  testGaitGeneratorABS --test9 --use-gnuplot false)
add_test(testGaitGeneratorTest10 testGaitGeneratorABS --test10 --use-gnuplot false)
add_test(testGaitGeneratorTest11 testGaitGeneratorABS --test11 --use-gnuplot false)
add_test(testGaitGeneratorTest12 testGaitGeneratorABS --test12 --use-gnuplot false)
add_test(testGaitGeneratorTest13 testGaitGeneratorABS --test13 --use-gnuplot false)
add_test(testGaitGeneratorTest14 testGaitGeneratorABS --test14 --use-gnuplot false)
add_test(testGaitGeneratorTest15 testGaitGeneratorABS --test15 --use-gnuplot false)
add_test(testGaitGeneratorTest16 testGaitGeneratorABS --test16 --use-gnuplot false)
add_test(testGaitGeneratorTest17 testGaitGeneratorABS --test17 --use-gnuplot false)
add_test(testGaitGeneratorTest18 testGaitGeneratorABS --test18 --use-gnuplot false)

add_test(testZMPDistributor_HRP2JSK_EEFM_Test0       testZMPDistributorABS --hrp2jsk --test0 --use-gnuplot false)
add_test(testZMPDistributor_HRP2JSK_EEFM_Test1       testZMPDistributorABS --hrp2jsk --test1 --use-gnuplot false)
add_test(testZMPDistributor_HRP2JSK_EEFM_Test2       testZMPDistributorABS --hrp2jsk --test2 --use-gnuplot false)
add_test(testZMPDistributor_JAXONRED_EEFM_Test0      testZMPDistributorABS --jaxon_red --test0 --use-gnuplot false)
add_test(testZMPDistributor_JAXONRED_EEFM_Test1      testZMPDistributorABS --jaxon_red --test1 --use-gnuplot false)
add_test(testZMPDistributor_JAXONRED_EEFM_Test2      testZMPDistributorABS --jaxon_red --test2 --use-gnuplot false)
add_test(testZMPDistributor_HRP2JSK_EEFMQPCOP_Test0  testZMPDistributorABS --hrp2jsk --test0 --use-gnuplot false --distribution-algorithm EEFMQPCOP)
add_test(testZMPDistributor_HRP2JSK_EEFMQPCOP_Test1  testZMPDistributorABS --hrp2jsk --test1 --use-gnuplot false --distribution-algorithm EEFMQPCOP)
add_test(testZMPDistributor_HRP2JSK_EEFMQPCOP_Test2  testZMPDistributorABS --hrp2jsk --test2 --use-gnuplot false --distribution-algorithm EEFMQPCOP)
add_test(testZMPDistributor_JAXONRED_EEFMQPCOP_Test0 testZMPDistributorABS --jaxon_red --test0 --use-gnuplot false --distribution-algorithm EEFMQPCOP)
add_test(testZMPDistributor_JAXONRED_EEFMQPCOP_Test1 testZMPDistributorABS --jaxon_red --test1 --use-gnuplot false --distribution-algorithm EEFMQPCOP)
add_test(testZMPDistributor_JAXONRED_EEFMQPCOP_Test2 testZMPDistributorABS --jaxon_red --test2 --use-gnuplot false --distribution-algorithm EEFMQPCOP)

set(target AutoBalanceStabilizer AutoBalanceStabilizerComp Stabilizer testPreviewControllerABS testGaitGeneratorABS testTwoDofControllerABS testZMPDistributorABS)

install(TARGETS ${target}
  RUNTIME DESTINATION bin
  LIBRARY DESTINATION lib
)
