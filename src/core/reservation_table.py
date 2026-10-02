from collections import defaultdict


class ReservationTable:
    """Tracks time-expanded vertex (hub) and edge (link) occupancy for drones.

    Enforces capacity limits across intermediate hubs and links while ensuring
    the start and goal hubs remain unconstrained.
    """

    def __init__(self, start_hub: str = "start", goal_hub: str = "goal"):
        self.start_hub = start_hub
        self.goal_hub = goal_hub

        # Maps (hub_name, time_step) -> drone_count
        self.hub_reservations: dict[tuple[str, int], int] = defaultdict(int)

        # Maps ((u, v), time_step) -> drone_count for transitions starting at time_step
        self.link_reservations: dict[tuple[tuple[str, str], int], int] = defaultdict(int)

    def is_hub_free(self, hub_name: str, time: int, max_capacity: int) -> bool:
        """Checks if a hub has available capacity at a specific time step.

        Start and Goal hubs always have infinite capacity.
        """
        if hub_name in (self.start_hub, self.goal_hub):
            return True
        return self.hub_reservations[(hub_name, time)] < max_capacity

    def is_link_free(
        self, u: str, v: str, time: int, max_capacity: int, bidirectional: bool = True
    ) -> bool:
        """Checks if a link between u and v has room during the transition tick 'time' -> 'time + 1'."""
        link_key = tuple(sorted([u, v])) if bidirectional else (u, v)
        return self.link_reservations[(link_key, time)] < max_capacity

    def reserve_path(
        self, path: list[str], max_horizon: int = 200, bidirectional: bool = True
    ) -> None:
        """Locks in a drone's space-time trajectory.

        - Reserves hub occupancy at time t = 0..len(path)-1.
        - Reserves link transitions from t-1 to t.
        - Keeps the drone parked at non-goal destinations up to max_horizon.
        """
        for t, hub_name in enumerate(path):
            self.hub_reservations[(hub_name, t)] += 1

            if t > 0:
                prev_hub = path[t - 1]
                link_key = tuple(sorted([prev_hub, hub_name])) if bidirectional else (prev_hub, hub_name)
                # Link traversal occurred during tick (t - 1) -> t
                self.link_reservations[(link_key, t - 1)] += 1

        # Park the drone at its destination up to max_horizon ONLY if it's not the goal hub
        last_hub = path[-1]
        last_time = len(path) - 1

        if last_hub != self.goal_hub:
            for t in range(last_time + 1, max_horizon):
                self.hub_reservations[(last_hub, t)] += 1