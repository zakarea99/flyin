from collections import deque
from src.core.graph import Graph
from src.model.ZoneType import ZoneType


class pathfinderr:
    """Multi-agent pathfinding solver using network flow node-splitting.

    Translates a physical graph of hubs into a directed residual network to
    compute vertex-disjoint and capacity-respecting paths using an
    Edmonds-Karp / Suurballe style flow augmentation with priority BFS.
    """

    def __init__(self, graph: Graph) -> None:
        """Initializes the solver and constructs the residual graph.

        Args:
            graph: The physical network Graph containing hubs and connections.
        """
        self.graph: Graph = graph
        self.residual_capacity: dict[tuple[str, str], int] = {}
        self.split_adj: dict[str, list[str]] = {}
        self.forward_edges: set[tuple[str, str]] = set()
        self.initial_capacity: dict[tuple[str, str], int] = {}
        self._initialize_residual_graph()

    def _add_edge(self, u: str, v: str, capacity: int) -> None:
        """Adds a directed forward edge and initializes its ghost edge.

        Args:
            u: Source split node name.
            v: Destination split node name.
            capacity: Forward capacity of the edge.
        """
        if u not in self.split_adj:
            self.split_adj[u] = []
        if v not in self.split_adj:
            self.split_adj[v] = []

        if v not in self.split_adj[u]:
            self.split_adj[u].append(v)
        if u not in self.split_adj[v]:
            self.split_adj[v].append(u)

        self.residual_capacity[(u, v)] = capacity
        self.residual_capacity[(v, u)] = 0
        self.initial_capacity[(u, v)] = capacity
        self.forward_edges.add((u, v))

    def _get_physical_hub_name(self, split_node: str) -> str:
        """Strips node-splitting suffixes to recover the physical hub name.

        Args:
            split_node: Split node identifier (e.g., 'hub_in', 'hub_wait').

        Returns:
            The underlying physical hub name string.
        """
        for suffix in ("_in", "_out", "_wait"):
            if split_node.endswith(suffix):
                return split_node[:-len(suffix)]
        return split_node

    def _get_hub_zone_type(self, hub_name: str) -> ZoneType:
        """Retrieves the ZoneType of a physical hub.

        Args:
            hub_name: Physical hub identifier.

        Returns:
            The ZoneType associated with the hub (defaults to normal).
        """
        hub = self.graph.hubs.get(hub_name)
        if hub is not None:
            if isinstance(hub.zone_type, ZoneType):
                return hub.zone_type
            return ZoneType(hub.zone_type)
        return ZoneType.normal

    def _initialize_residual_graph(self) -> None:
        """Translates physical undirected graph into directed residual network.

        - Normal / Priority hubs split into Hub_in -> Hub_out.
        - Restricted hubs split into Hub_in -> Hub_wait -> Hub_out.
        - Start and End hubs receive infinite (massive integer) capacity.
        - Physical undirected connections (A, B) create directed external edges
          A_out -> B_in and B_out -> A_in with max_link_capacity.
        - Ghost edges (v, u) are initialized with 0 capacity.
        """
        self.residual_capacity.clear()
        self.split_adj.clear()
        self.forward_edges.clear()
        self.initial_capacity.clear()

        inf_capacity: int = 1_000_000_000

        # 1. Internal node-splitting edges
        for hub_name, hub in self.graph.hubs.items():
            is_terminal = (
                hub_name == self.graph.start_hub
                or hub_name == self.graph.end_hub
                or getattr(hub, "is_start", False)
                or getattr(hub, "is_end", False)
            )

            if is_terminal:
                self._add_edge(
                    f"{hub_name}_in", f"{hub_name}_out", inf_capacity
                )
            else:
                zone = self._get_hub_zone_type(hub_name)
                cap = hub.max_drones
                if zone == ZoneType.restricted:
                    self._add_edge(f"{hub_name}_in", f"{hub_name}_wait", cap)
                    self._add_edge(f"{hub_name}_wait", f"{hub_name}_out", cap)
                elif zone == ZoneType.blocked:
                    continue
                else:
                    self._add_edge(f"{hub_name}_in", f"{hub_name}_out", cap)

        # 2. External directed connection edges
        visited_connections: set[tuple[str, str]] = set()
        for u, neighbors in self.graph.adjacency_list.items():
            for v in neighbors:
                if (
                    self._get_hub_zone_type(u) == ZoneType.blocked
                    or self._get_hub_zone_type(v) == ZoneType.blocked
                ):
                    continue
                edge_pair = (min(u, v), max(u, v))
                if edge_pair in visited_connections:
                    continue
                visited_connections.add(edge_pair)

                cap = self.graph.connections_capacities.get((u, v), 1)
                self._add_edge(f"{u}_out", f"{v}_in", cap)
                self._add_edge(f"{v}_out", f"{u}_in", cap)

    def _solve_bfs(self) -> list[str]:
        """Finds an augmenting path from start_out to end_in using priority BFS.

        Evaluates neighbors using three deque queues in strict priority order:
        Priority > Normal > Restricted.

        Returns:
            A list of split node names from start_out to end_in, or empty list.
        """
        start_node = f"{self.graph.start_hub}_out"
        end_node = f"{self.graph.end_hub}_in"

        if start_node == end_node:
            return [start_node]

        queue_priority: deque[tuple[str, list[str]]] = deque()
        queue_normal: deque[tuple[str, list[str]]] = deque()
        queue_restricted: deque[tuple[str, list[str]]] = deque()

        start_phys = self._get_physical_hub_name(start_node)
        start_zone = self._get_hub_zone_type(start_phys)

        if start_zone == ZoneType.priority:
            queue_priority.append((start_node, [start_node]))
        elif start_zone == ZoneType.restricted:
            queue_restricted.append((start_node, [start_node]))
        else:
            queue_normal.append((start_node, [start_node]))

        visited: set[str] = {start_node}

        while queue_priority or queue_normal or queue_restricted:
            if queue_priority:
                hub, path = queue_priority.popleft()
            elif queue_normal:
                hub, path = queue_normal.popleft()
            else:
                hub, path = queue_restricted.popleft()

            for neighbor in self.split_adj.get(hub, []):
                if (
                    neighbor not in visited
                    and self.residual_capacity.get((hub, neighbor), 0) > 0
                ):
                    new_path = path + [neighbor]
                    if neighbor == end_node:
                        return new_path

                    visited.add(neighbor)
                    phys_name = self._get_physical_hub_name(neighbor)
                    zone_type = self._get_hub_zone_type(phys_name)

                    if zone_type == ZoneType.priority:
                        queue_priority.append((neighbor, new_path))
                    elif zone_type == ZoneType.restricted:
                        queue_restricted.append((neighbor, new_path))
                    else:
                        queue_normal.append((neighbor, new_path))

        return []

    def _apply_path_and_reverse_edges(self, path: list[str]) -> None:
        """Applies a unit of flow along the BFS path and updates ghost edges.

        Args:
            path: Sequence of split node names representing the augmenting path.
        """
        for i in range(len(path) - 1):
            u = path[i]
            v = path[i + 1]
            self.residual_capacity[(u, v)] -= 1
            self.residual_capacity[(v, u)] += 1

    def extract_all_disjoint_paths(self) -> list[list[str]]:
        """Main loop finding all disjoint paths until residual capacity is exhausted.

        Returns:
            A list of paths, each represented as a list of physical hub names.
        """
        while True:
            path = self._solve_bfs()
            if not path:
                break
            self._apply_path_and_reverse_edges(path)
        return self._untangle_paths()

    def _untangle_paths(self) -> list[list[str]]:
        """Untangles overlapping flow paths by tracing saturated flow edges.

        Traverses edges where forward capacity is 0 (or carrying active net
        flow) and the reverse ghost edge capacity is not 0 to eliminate
        cancelled crossovers.

        Returns:
            A clean list of paths containing only physical hub names.
        """
        start_node = f"{self.graph.start_hub}_out"
        end_node = f"{self.graph.end_hub}_in"

        active_adj: dict[str, list[str]] = {}
        for (u, v) in self.forward_edges:
            ghost_cap = self.residual_capacity.get((v, u), 0)
            forward_cap = self.residual_capacity.get((u, v), 0)
            init_cap = self.initial_capacity.get((u, v), 0)

            # Saturated or positive active net flow that was not cancelled
            if ghost_cap != 0 and (forward_cap == 0 or forward_cap < init_cap):
                if u not in active_adj:
                    active_adj[u] = []
                for _ in range(ghost_cap):
                    active_adj[u].append(v)

        for u in active_adj:
            active_adj[u].sort()

        paths: list[list[str]] = []

        def _find_path_dfs(
            curr: str, target: str, visited_in_path: set[str]
        ) -> list[str] | None:
            if curr == target:
                return [curr]
            visited_in_path.add(curr)
            for neighbor in list(active_adj.get(curr, [])):
                if neighbor not in visited_in_path:
                    res = _find_path_dfs(neighbor, target, visited_in_path)
                    if res is not None:
                        return [curr] + res
            visited_in_path.remove(curr)
            return None

        while True:
            visited_in_path: set[str] = set()
            raw_path = _find_path_dfs(start_node, end_node, visited_in_path)
            if not raw_path:
                break

            for i in range(len(raw_path) - 1):
                u_node = raw_path[i]
                v_node = raw_path[i + 1]
                if v_node in active_adj.get(u_node, []):
                    active_adj[u_node].remove(v_node)

            physical_path: list[str] = []
            for split_node in raw_path:
                hub_name = self._get_physical_hub_name(split_node)
                if not physical_path or physical_path[-1] != hub_name:
                    physical_path.append(hub_name)

            paths.append(physical_path)

        return paths
