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
#

from src.parser.perse import Parser
from core.graph1 import Graph
from src.core.reservation_table import ReservationTable
from src.core.pathfinder import SpaceTimePathfinder
from src.visualization.visualizer import PygameVisualizer


def main():
    map_file = "maps/challenger/01_the_impossible_dream.txt"
    parser = Parser()
    raw_hubs, raw_connections, nb_drones = parser.parse(map_file)

    graph = Graph(raw_hubs, raw_connections)
    start_hub = next(h.name for h in raw_hubs if h.is_start)
    end_hub = next(h.name for h in raw_hubs if h.is_end)

    reservation_table = ReservationTable(start_hub=start_hub, goal_hub=end_hub)
    pathfinder = SpaceTimePathfinder(graph, reservation_table)

    all_drone_paths: dict[int, list[str]] = {}

    for drone_id in range(1, nb_drones + 1):
        path = pathfinder.find_path(start_hub, end_hub)
        if path is None:
            print(f"Error: Could not route Drone {drone_id}")
            break
        reservation_table.reserve_path(path)
        all_drone_paths[drone_id] = path

    print(f"Solved map in {max(len(p) for p in all_drone_paths.values()) - 1} turns.")

    # --- LAUNCH PYGAME VISUALIZER ---
    visualizer = PygameVisualizer(graph, all_drone_paths)
    visualizer.run()


if __name__ == "__main__":
    main()