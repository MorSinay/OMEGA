cmake_minimum_required(VERSION 3.12.2)

project(MonteCarloSolver)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED True)

set(CMAKE_CONFIGURATION_TYPES "Debug;Release"
    CACHE STRING "Configuration types" FORCE)

add_executable(MonteCarloSolver
  ../../DroneSim/Agents/MosheAgent/MosheSearchModule.cpp
  ../../DroneSim/Agents/MosheAgent/MosheSearchModule.h
  ../../DroneSim/defines.h
  ../../DroneSim/Entities/Drone.cpp
  ../../DroneSim/Entities/Drone.h
  ../../DroneSim/Entities/Enemy.cpp
  ../../DroneSim/Entities/Enemy.h
  ../../DroneSim/Entities/EnemyEntity.cpp
  ../../DroneSim/Entities/EnemyEntity.h
  ../../DroneSim/MilitarySimulator.cpp
  ../../DroneSim/MilitarySimulator.h
  ../../DroneSim/Structures/Section.cpp
  ../../DroneSim/Structures/Section.h
  ../../DroneSim/types.h
  # ActionInfo.cpp
  # ActionInfo.h
  Console.cpp
  Console.h
  MonteCarloSolver.cpp
  # Node.cpp
  # Node.h
  # Observation.cpp
  # Observation.h
  # Point.cpp
  # Point.h
  # Random.cpp
  # Random.h
  # Solver.cpp
  # Solver.h
  # State.cpp
  # State.h
)
