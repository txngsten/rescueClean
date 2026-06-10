# rescueClean COMP3712
This README is for Oliver Wuttke's (WUTT0019) code solution for the major assignment for Advanced Algorithms and Programming COMP3712.
The JVM runtime being used for all testing and development was the OpenJDK 26.0.1.

## Solution Package Structure
The solution package is broken down into subpackages and contains a lot of Java files.
```
src/solution/
├── BfsResponder.java
├── DStarResponder.java
├── DijkstraResponder.java
├── DisasterResponder.java
├── Edge.java
├── Graph.java
├── GraphBuilder.java
├── HybridResponder.java
├── MissionStats.java
├── MyDisasterResponder.java
├── VehicleManager.java
└── Pathfinding/
    ├── PathResult.java
    ├── PathTask.java
    ├── RoutingEngine.java
    └── Algorithms/
        ├── BfsPathfinder.java
        ├── DStarLite.java
        ├── DfsPathfinder.java
        ├── DijkstraPathfinder.java
        ├── IncrementalPathfinder.java
        └── Pathfinder.java
```

The [MyDisasterResponder.java](src/solution/MyDisasterResponder.java) acts as the central command and control for delegating tasks to other parts of the system.
All the different pathfinding algorithms can be found [here](src/solution/Pathfinding/Algorithms).

## How to use
The only thing you need to change between implementations is the [sim.cfg](cfg/sim.cfg) file and only the RESPONDER_CLASS parameter.
For the [Hybrid Responder](src/solution/HybridResponder.java) which uses Dijkstra's algorithm for outbound path computation and D* Lite for return to base computations set the RESPONDER_CLASS parameter as such:
```properties
RESPONDER_CLASS=solution.HybridResponder
```
Then for the [BFS Responder](src/solution/BfsResponder.java) which uses BFS for both pathfinding computations:
```properties
RESPONDER_CLASS=solution.BfsResponder
```
Then for [D* Lite](src/solution/DStarResponder.java) which uses D* Lite for both pathfinding computations:
```properties
RESPONDER_CLASS=solution.DStarResponder
```
Then finally for [Dijkstra's Algorithm Responder](src/solution/DijkstraResponder.java) which uses Dijkstra's algorithm for both computations:
```properties
RESPONDER_CLASS=solution.DijkstraResponder
```
