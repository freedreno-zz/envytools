project(ENVYTOOLS C)
cmake_minimum_required(VERSION 2.6)

# <3 libxml2
SET(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Wno-pointer-sign")

find_package(LibXml2 REQUIRED)
find_package(Curses REQUIRED)

find_package(PkgConfig)
pkg_check_modules(LIBCONFIG REQUIRED libconfig)
pkg_check_modules(LIBDRM REQUIRED libdrm_freedreno)

find_path(LIBICONV_INCLUDE_DIR iconv.h)
include_directories(${LIBXML2_INCLUDE_DIR} ${LIBICONV_INCLUDE_DIR} ${CURSES_INCLUDE_DIR} ${LIBDRM_INCLUDE_DIRS})
link_directories(${LIBCONFIG_LIBRARY_DIRS} ${LIBDRM_LIBRARY_DIRS})

configure_file(rnn_path.h.in rnn_path.h ESCAPE_QUOTES)
include_directories(${PROJECT_BINARY_DIR})

add_library(rnn rnn.c rnndec.c)

add_executable(demmio demmio.c)
add_executable(demsm demsm.c)
add_executable(headergen headergen.c)
add_executable(headergen2 headergen2.c)
add_executable(dedma dedma.c dedma_cache.c dedma_back.c)
add_executable(lookup lookup.c)
add_executable(rnncheck rnncheck.c)
add_executable(fdperf fdperf.c)

target_link_libraries(rnn ${LIBXML2_LIBRARIES} envyutil)
target_link_libraries(demmio envy rnn)
target_link_libraries(demsm rnn)
target_link_libraries(headergen rnn)
target_link_libraries(headergen2 rnn)
target_link_libraries(dedma rnn)
target_link_libraries(lookup rnn)
target_link_libraries(rnncheck rnn)
target_link_libraries(fdperf ${CURSES_LIBRARIES} ${LIBCONFIG_LIBRARIES} ${LIBDRM_LIBRARIES} rnn)

install(TARGETS demmio demsm headergen headergen2 rnn dedma lookup fdperf
	RUNTIME DESTINATION bin
	LIBRARY DESTINATION lib${LIB_SUFFIX}
	ARCHIVE DESTINATION lib${LIB_SUFFIX})

add_test(check_nv_mmio rnncheck nv_mmio.xml)
add_test(check_nv_evo rnncheck nv_evo.xml)
add_test(check_nv_structs rnncheck nv_struct.xml)
add_test(check_nv_objects rnncheck nv_objects.xml)
add_test(check_nv50_texture rnncheck nv50_texture.xml)
add_test(check_adt7473 rnncheck extdev/adt7473.xml)
add_test(check_nv17_mpeg rnncheck nv17_mpeg.xml)
add_test(check_nvc0_shaders rnncheck nvc0_shaders.xml)
