all:
	g++ -c Structures/Section.cpp -o Section.o
	g++ -c utils/Dijkstra.cpp -o Dijkstra.o
	g++ -c Entities/Enemy.cpp -o Enemy.o
	g++ -c Entities/UGV.cpp -o UGV.o
	g++ -c Entities/Drone.cpp -o Drone.o
	g++ -c MilitarySimulator.cpp -o MilitarySimulator.o
	g++ main.cpp MilitarySimulator.o UGV.o Drone.o Dijkstra.o Section.o Enemy.o

# ugv:
# 	g++ -c utils/Dijkstra.cpp -o Dijkstra.o
# 	g++ -c Structures/Section.cpp -o Section.o
# 	g++ -c Entities/UGV.cpp -o UGV.o
# 	g++ -c MilitarySimulator.cpp -o MilitarySimulator.o
# 	g++ main.cpp MilitarySimulator.o UGV.o Dijkstra.o
