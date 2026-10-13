from src.model.Connection import Connection
from src.model.Hub import Hub
from math import hypot


class Graph:
    def __init__(self, hubs: list[Hub], connections: list[Connection]):
        self.hubs: dict[str, Hub] = {
            hub.name: hub for hub in hubs}
        self.adjacency_list: dict[str, list[str]] = {
            hub.name: [] for hub in hubs}
        self.connections_capacities: dict[tuple[str, str], int] = {}
        self.start_hub: str = ""
        self.end_hub: str = ""
        self._identify_terminals()
        self._build_graph(connections)

    def _identify_terminals(self) -> None:
        """Finds and stores the start and end hub names."""
        for name, hub in self.hubs.items():
            if hub.is_start:
                self.start_hub = name
            elif hub.is_end:
                self.end_hub = name
        if not self.start_hub or not self.end_hub:
            raise ValueError(
                "Graph Error: Graph must have exactly \
                    one start and one end hub.")

    def _build_graph(self, connections: list[Connection]) -> None:
        for con in connections:
            x = con.from_hub
            y = con.to_hub
            if x not in self.hubs or y not in self.hubs:
                raise ValueError(f"Graph Error: Connection \
                                  references missing hub '{x}' or '{y}'")
            if y not in self.adjacency_list[x]:
                self.adjacency_list[x].append(y)
                self.adjacency_list[y].append(x)
            self.connections_capacities[(x, y)] = con.max_link_capacity
            self.connections_capacities[(y, x)] = con.max_link_capacity

    def get_neighbors(self, hub_name: str) -> list[str]:
        """Returns the adjacent hubs for a given node."""
        return self.adjacency_list.get(hub_name, [])

    def get_hub_type(self, hub_name: str) -> str:
        """Rerturn the type of the hub"""
        hubtype = self.hubs.get(hub_name)
        return hubtype.zone_type

    def get_link_capacity(self, hub1: str, hub2: str) -> int:
        """Returns the edge capacity between two hubs, 0 if no link exists."""
        return self.connections_capacities.get((hub1, hub2), 0)

    def get_node_capacity(self, hub_name: str) -> int:
        """
        Returns the node capacity.
        """
        if hub_name == self.start_hub or hub_name == self.end_hub:
            return 2000000
        return self.hubs[hub_name].max_drones

    def get_distance(self, hub1_name: str, hub2_name: str) -> float:
        """Calculates the Euclidean distance for A* heuristics."""
        h1 = self.hubs[hub1_name]
        h2 = self.hubs[hub2_name]
        return hypot(h1.x - h2.x, h1.y - h2.y)

    def get_coordinates(self, hub_name: str) -> tuple[int, int]:
        """Returns the (x, y) coordinates of a specific hub."""
        hub = self.hubs.get(hub_name)
        if not hub:
            raise KeyError(f"Graph Error: Hub '{hub_name}' does not exist.")
        return (hub.x, hub.y)
