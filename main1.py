import sys
from src.parser.perse import Parser
from src.core.graph import Graph
from src.core.pathfinderr import pathfinderr
from src.core.reservation_table import ReservationTable
from src.core.simulator import SimulationPrinter
from src.model.ZoneType import ZoneType


def schedule_drone_on_path(
    path: list[str],
    table: ReservationTable,
    graph: Graph,
) -> list[str]:
    """Schedules a single drone along a pre-computed disjoint path.

    Advances the drone at each time step if link and target hub capacities
    allow; otherwise, keeps the drone waiting at the current hub until
    congestion clears.

    If the destination is a restricted zone, movement requires 2 turns
    (1 turn to transit/enter, and 1 turn waiting in the zone).

    Args:
        path: Ordered list of physical hub names from start to goal.
        table: Reservation table tracking time-expanded hub and link usage.
        graph: Physical network graph.

    Returns:
        The space-time trajectory of hub names at each time tick (t = 0, 1, ...).
    """
    start_hub = path[0]
    st_path: list[str] = [start_hub]
    t = 0

    for idx in range(1, len(path)):
        target = path[idx]
        target_obj = graph.hubs[target]
        is_restricted = (
            target_obj.zone_type == ZoneType.restricted
            and target != graph.end_hub
        )

        while True:
            curr = st_path[-1]
            max_link = graph.connections_capacities.get((curr, target), 1)
            target_cap = graph.get_node_capacity(target)

            link_free = table.is_link_free(curr, target, t, max_link)
            hub_free_t1 = table.is_hub_free(target, t + 1, target_cap)

            if is_restricted:
                # Must ensure capacity is available for both arrival and wait turns
                hub_free_t2 = table.is_hub_free(target, t + 2, target_cap)
                can_move = link_free and hub_free_t1 and hub_free_t2
            else:
                can_move = link_free and hub_free_t1

            if can_move:
                # Move into target hub
                st_path.append(target)
                t += 1

                # Restricted zone: drone must wait 1 turn in place (2 turns total)
                if is_restricted:
                    st_path.append(target)
                    t += 1
                break
            else:
                # Congested: drone waits at the current hub for this turn
                st_path.append(curr)
                t += 1

    return st_path


def solve_and_simulate(
    map_file: str,
    show_simulation_turns: bool = True,
) -> tuple[int, dict[int, list[str]], list[list[str]]]:
    """Solves the map using pathfinderr and schedules all drones.

    Args:
        map_file: Path to the input map file.
        show_simulation_turns: Whether to print the turn-by-turn simulation moves.

    Returns:
        A tuple of (total_turns, all_drone_paths, disjoint_paths).
    """
    parser = Parser()
    raw_hubs, raw_connections, nb_drones = parser.parse(map_file)
    graph = Graph(raw_hubs, raw_connections)

    start_hub = next(h.name for h in raw_hubs if h.is_start)
    end_hub = next(h.name for h in raw_hubs if h.is_end)

    print(f"=== Solved with pathfinderr: {map_file} ===")
    print(f"Drones to route: {nb_drones}")
    print(f"Start Hub: {start_hub} | End Hub: {end_hub}\n")

    # 1. Run Edmonds-Karp / Suurballe solver to extract disjoint paths
    solver = pathfinderr(graph)
    disjoint_paths = solver.extract_all_disjoint_paths()

    print(f"Disjoint path(s) extracted: {len(disjoint_paths)}")
    for i, p in enumerate(disjoint_paths, 1):
        print(f"  Path {i} ({len(p) - 1} steps, {len(p)} hubs): {' -> '.join(p)}")
    print()

    if not disjoint_paths:
        print("Error: No paths could be found from start to goal.")
        return 0, {}, []

    # 2. Schedule all drones into space-time pipelines
    table = ReservationTable(start_hub=start_hub, goal_hub=end_hub)
    all_drone_paths: dict[int, list[str]] = {}

    for drone_id in range(1, nb_drones + 1):
        best_trajectory: list[str] | None = None

        # Choose the path that arrives earliest for this drone
        for candidate_path in disjoint_paths:
            traj = schedule_drone_on_path(candidate_path, table, graph)
            if best_trajectory is None or len(traj) < len(best_trajectory):
                best_trajectory = traj

        if best_trajectory is None:
            print(f"Error: Could not schedule Drone {drone_id}")
            break

        table.reserve_path(best_trajectory)
        all_drone_paths[drone_id] = best_trajectory

    total_turns = max(len(p) for p in all_drone_paths.values()) - 1

    if show_simulation_turns:
        SimulationPrinter.print_turn_by_turn(all_drone_paths)

    return total_turns, all_drone_paths, disjoint_paths


def main() -> None:
    map_file = "maps/challenger/01_the_impossible_dream.txt"
    if len(sys.argv) > 1 and not sys.argv[1].startswith("--"):
        map_file = sys.argv[1]

    visual = "--visual" in sys.argv

    turns, all_drone_paths, paths = solve_and_simulate(
        map_file=map_file,
        show_simulation_turns=True,
    )

    print("\n" + "=" * 50)
    print(f"RESULT: Solved {map_file} in {turns} turns!")
    print("=" * 50 + "\n")

    if visual and all_drone_paths:
        try:
            from src.visualization.visualizer import PygameVisualizer
            parser = Parser()
            raw_hubs, raw_connections, _ = parser.parse(map_file)
            graph = Graph(raw_hubs, raw_connections)
            visualizer = PygameVisualizer(graph, all_drone_paths)
            visualizer.run()
        except Exception as e:
            print(f"Could not launch Pygame visualizer: {e}")


if __name__ == "__main__":
    main()
