cmake_minimum_required(VERSION 2.8.3)
project(baxter_moveit)
find_package(catkin REQUIRED)
catkin_metapackage()

foreach(dir launch urdf worlds)
    install(
       DIRECTORY ${dir}/
       DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/${dir}
       USE_SOURCE_PERMISSIONS
    )
endforeach(dir)
