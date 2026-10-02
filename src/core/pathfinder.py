import heapq
from src.core.graph import Graph
from src.core.reservation_table import ReservationTable
from src.model.ZoneType import ZoneType


class SpaceTimePathfinder:
    def __init__(self, graph: Graph, reservation_table: ReservationTable):
        self.graph = graph
        self.reservation_table = reservation_table

    def find_path(
        self,
        start_name: str,
        end_name: str,
        start_time: int = 0,
        max_time_horizon: int = 200
    ) -> list[str] | None:
        """Calculates a collision-free path through space and time."""
        
        # Priority queue stores: (f_score, g_score, hub_name, time_step)
        open_set: list[tuple[float, float, str, int]] = []
        
        h_start = self.graph.get_distance(start_name, end_name)
        heapq.heappush(open_set, (h_start, 0.0, start_name, start_time))

        g_score: dict[tuple[str, int], float] = {(start_name, start_time): 0.0}
        came_from: dict[tuple[str, int], tuple[str, int]] = {}

        while open_set:
            _, current_g, current_hub, t = heapq.heappop(open_set)

            # Goal reached!
            if current_hub == end_name:
                return self._reconstruct_path(came_from, (current_hub, t))

            # Safety cutoff to prevent infinite searches
            if t >= max_time_horizon:
                continue

            # ---------------------------------------------------------
            # ACTION 1: Move to an adjacent hub at tick t + 1
            # ---------------------------------------------------------
            for neighbor in self.graph.adjacency_list[current_hub]:
                neighbor_hub = self.graph.hubs[neighbor]

                # 1. Skip blocked zones entirely
                if neighbor_hub.zone_type == ZoneType.blocked:
                    continue

                # 2. Check connection capacity for transition t -> t+1
                max_link_cap = self.graph.connections_capacities.get((current_hub, neighbor), 1)
                if not self.reservation_table.is_link_free(current_hub, neighbor, t, max_link_cap):
                    continue

                # 3. Check target hub capacity at tick t+1
                if not self.reservation_table.is_hub_free(neighbor, t + 1, neighbor_hub.max_drones):
                    continue

                # Compute step cost with zone penalties
                dist = self.graph.get_distance(current_hub, neighbor)
                if neighbor_hub.zone_type == ZoneType.restricted:
                    dist *= 5.0
                elif neighbor_hub.zone_type == ZoneType.priority:
                    dist *= 0.5

                tentative_g = current_g + dist
                next_state = (neighbor, t + 1)

                if tentative_g < g_score.get(next_state, float("inf")):
                    g_score[next_state] = tentative_g
                    came_from[next_state] = (current_hub, t)
                    f_score = tentative_g + self.graph.get_distance(neighbor, end_name)
                    heapq.heappush(open_set, (f_score, tentative_g, neighbor, t + 1))

            # ---------------------------------------------------------
            # ACTION 2: Wait at current_hub at tick t + 1
            # ---------------------------------------------------------
            current_hub_obj = self.graph.hubs[current_hub]
            if self.reservation_table.is_hub_free(current_hub, t + 1, current_hub_obj.max_drones):
                wait_cost = 1.0  # Small time penalty for waiting
                tentative_g = current_g + wait_cost
                next_state = (current_hub, t + 1)

                if tentative_g < g_score.get(next_state, float("inf")):
                    g_score[next_state] = tentative_g
                    came_from[next_state] = (current_hub, t)
                    f_score = tentative_g + self.graph.get_distance(current_hub, end_name)
                    heapq.heappush(open_set, (f_score, tentative_g, current_hub, t + 1))

        return None  # No path found

    def _reconstruct_path(
        self,
        came_from: dict[tuple[str, int], tuple[str, int]],
        current_state: tuple[str, int]
    ) -> list[str]:
        """Extracts the sequence of hub names from space-time states."""
        path = [current_state[0]]
        curr = current_state
        while curr in came_from:
            curr = came_from[curr]
            path.append(curr[0])
        path.reverse()
        return path