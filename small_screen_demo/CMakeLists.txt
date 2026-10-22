message(STATUS "cmake config ${CMAKE_CURRENT_SOURCE_DIR}")

# 外部应用工程：小屏点击计数器 demo
set(target small_screen_demo)

# 将本工程产物注册进最终固件链接列表
add_apps_libraries($<TARGET_FILE:${target}>)

add_library(${target} STATIC)
set_target_properties(${target} PROPERTIES ARCHIVE_OUTPUT_DIRECTORY ${out_unir_lib_dir})

target_sources(${target} PRIVATE
    "small_screen.c"
    "fish_img_muyu.c"
    "feature_icons.c"
)

# 链接 UniRTOS 系统库，获取 qosa_lcd / qosa_iic / qosa_gpio / qosa_sys 等 HAL 接口
target_link_libraries_if(CONFIG_QOSA_LINK_SYSTEM_LIBRARIES_FUNC THEN ${target} system_os)

# 链接 unirtos-lvgl 外部库（版本由 env_config.json 的 libraries.list 声明，
# SDK 顶层 CMakeLists.txt 的 discover_and_link_external_libraries() 会据此
# 自动 add_subdirectory 到 ~/.unirtos/libraries/unirtos-lvgl/v8.3.11/ 并定义
# 同名 target，这里直接 target_link_libraries 即可拿到其 PUBLIC include 路径
# （lv_conf.h + lvgl-v8.3.11/）和 LV_CONF_INCLUDE_SIMPLE 编译定义）
target_link_libraries(${target} PRIVATE unirtos-lvgl)
