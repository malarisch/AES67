# SPDX-License-Identifier: Apache-2.0

# auracast_card_proto.c is compiled by the app CMakeLists (card_settings
# needs it even without the driver).
target_sources(app PRIVATE auracast_card.c)
target_include_directories(app PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})
