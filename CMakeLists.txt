cmake_minimum_required(VERSION 3.10)
project(content_aware_image_resize)

set(CMAKE_CXX_STANDARD 17)

add_compile_options(-Wall -Wextra -Werror)

find_package(OpenCV REQUIRED)

add_executable(content_aware_image_resize main.cpp)
target_include_directories(content_aware_image_resize PRIVATE ${OpenCV_INCLUDE_DIRS})
target_include_directories(content_aware_image_resize PUBLIC include)
target_link_libraries(content_aware_image_resize ${OpenCV_LIBS})

add_executable(seam_carving_test test.cpp)
target_include_directories(seam_carving_test PRIVATE ${OpenCV_INCLUDE_DIRS})
target_include_directories(seam_carving_test PRIVATE include)
target_link_libraries(seam_carving_test ${OpenCV_LIBS})
