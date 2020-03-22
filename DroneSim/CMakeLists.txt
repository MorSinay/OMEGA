
cmake_minimum_required(VERSION 3.12.2)

project(DroneSim)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED True)
# set(CMAKE_CXX_EXTENSIONS False)

set(CMAKE_CONFIGURATION_TYPES "Debug;Release"
    CACHE STRING "Configuration types" FORCE)

add_executable(DroneSim
  Agents/MosheAgent/MosheSearchModule.cpp
  Agents/MosheAgent/MosheSearchModule.h
  defines.h
  Entities/Drone.cpp
  Entities/Drone.h
  Entities/Enemy.cpp
  Entities/Enemy.h
  Entities/EnemyEntity.cpp
  Entities/EnemyEntity.h
  main.cpp
  MilitarySimulator.cpp
  MilitarySimulator.h
  Structures/Section.cpp
  Structures/Section.h
  types.h
)
