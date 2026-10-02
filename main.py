# from src.parser.perse import Parser
# from core.graph import Graph


# def main():
#     filepath = "maps/hard/03_ultimate_challenge.txt"

#     print("--- 1. PARSING MAP ---")
#     parser = Parser()
#     raw_hubs, raw_connections, nb_drones = parser.parse(filepath)
#     print(f"Parsed {len(raw_hubs)} hubs and {len(raw_connections)} connections.")
#     print(f"Total drones available: {nb_drones}")

#     print("\n--- 2. BUILDING GRAPH ---")
#     drone_network = Graph(raw_hubs, raw_connections)
#     print("Graph built successfully!")

#     print("\n--- 3. GRAPH DATA CHECK ---")
#     start_hub = next(h.name for h in raw_hubs if h.is_start)
#     print(f"Start Hub: {start_hub}")
#     print(f"Neighbors of Start: {drone_network.adjacency_list[start_hub]}")
#     if drone_network.adjacency_list[start_hub]:
#         first_neighbor = drone_network.adjacency_list[start_hub][0]
#         capacity = drone_network.connections_capacities[(start_hub, first_neighbor)]
#         print(f"Capacity from {start_hub} to {first_neighbor}: {capacity}")


# if __name__ == "__main__":
#     main()
from src.core.graph import Graph
from src.core.pathfinder import SpaceTimePathfinder
from src.core.reservation_table import ReservationTable
from src.parser.perse import Parser
from src.core.simulator import SimulationPrinter

def main():
    parser = Parser()
    raw_hubs, raw_connections, nb_drones = parser.parse("maps/challenger/01_the_impossible_dream.txt")
    graph = Graph(raw_hubs, raw_connections)

    start_hub = next(h.name for h in raw_hubs if h.is_start)
    end_hub = next(h.name for h in raw_hubs if h.is_end)

    reservation_table = ReservationTable()
    pathfinder = SpaceTimePathfinder(graph, reservation_table)

    all_drone_paths: dict[int, list[str]] = {}

    print(f"--- ROUTING {nb_drones} DRONES ---")
    for drone_id in range(1, nb_drones + 1):
        path = pathfinder.find_path(start_hub, end_hub)

        if path is None:
            print(f"Error: Could not find a valid route for Drone {drone_id}!")
            break

        # Reserve space-time slots so later drones avoid collisions
        reservation_table.reserve_path(path)
        all_drone_paths[drone_id] = path

        print(f"Drone {drone_id} Path ({len(path)} steps): {' -> '.join(path)}")
    SimulationPrinter.print_turn_by_turn(all_drone_paths)


if __name__ == "__main__":
    main()