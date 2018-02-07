cmake_minimum_required(VERSION 2.8.3)
project(ros_toon)

#To avoid problem on early check of existance of this directory
file(MAKE_DIRECTORY ${CATKIN_DEVEL_PREFIX}/include)

find_package(catkin REQUIRED)
find_package(LAPACK REQUIRED)


include(ExternalProject)
ExternalProject_Add(upstream_igh_eml    # Name for custom target
# #--Download step--------------
  GIT_REPOSITORY https://github.com/edrosten/TooN.git 
  GIT_TAG d3685e80c5fbeeea18a61cbabc3cfa0c6cf93a3a
  TIMEOUT 30           # Time allowed for file download operations
# #--Configure step-------------
  CONFIGURE_COMMAND <SOURCE_DIR>/configure --prefix=<INSTALL_DIR>
# #--Build step-----------------
  BUILD_IN_SOURCE 1         # Use source dir for build dir
# #--Install step---------------
  INSTALL_DIR ${CATKIN_DEVEL_PREFIX}           # Installation prefix
  )

#info that we give to ros packages that depend on this one
#It is a header-only library, so we only pass the include directory
catkin_package(
  INCLUDE_DIRS ${CATKIN_DEVEL_PREFIX}/include
  DEPENDS LAPACK
)

