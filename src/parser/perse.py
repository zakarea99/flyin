from typing import Any
from src.model.ZoneType import ZoneType
from src.model.Connection import Connection
from src.model.Hub import Hub
# i should check that Hub name should not contain a space or - done
# i should check that no repeted Hubs or Conection! -done
#if the graph is not directed i have to change the depilcats check

class Parser:
    def _parse_optional_part(self, optional_info: str) -> dict:
        """Parses the metadata brackets and returns a dict of kwargs."""
        kwargs: dict[str, Any] = {}
        for part in optional_info.split():
            if "=" in part:
                key, value = part.split("=", 1)
                key, value = key.strip(), value.strip()
                if key == "color":
                    kwargs["color"] = value
                elif key == "max_drones":
                    kwargs["max_drones"] = int(value)
                elif key == "zone":
                    kwargs["zone_type"] = ZoneType(value)
        return kwargs

    def _parse_hub_line(self, line_data: str, **flags) -> Hub:
        """Shared logic to create a Hub from a data string."""
        kwargs: dict[str, Any] = {}
        if "[" in line_data and "]" in line_data:
            base_info, meta_info = line_data.split("[", 1)
            meta_string = meta_info.replace("]", "").strip()
            kwargs = self._parse_optional_part(meta_string)
        else:
            base_info = line_data
        parts = base_info.strip().split()
        if len(parts) != 3:
            raise ValueError(
                f"Parse Error on '{base_info}': Expected exactly 3 parts "
                f"(name, x, y). Ensure coordinates are present and the name"
                f" contains NO spaces."
            )
        name = parts[0].strip()
        if "-" in name:
            raise ValueError(f"Parse Error: Hub name "
                             f"'{name}' contains a dash.")
        return Hub(
            name=name,
            x=int(parts[1]),
            y=int(parts[2]),
            **kwargs,
            **flags
        )

    def _conection_lines_sep(self, line: str) -> Connection:
        capacity_str = "1"
        if "[" in line and "]" in line:
            base_info, meta_info = line.split("[", 1)
            meta_string = meta_info.replace("]", "").strip()
            if "max_link_capacity=" in meta_string:
                capacity_str = meta_string.split("=")[1].strip()
        else:
            base_info = line
        from_hub, to_hub = base_info.strip().split("-")
        return Connection(
            from_hub=from_hub.strip(),
            to_hub=to_hub.strip(),
            max_link_capacity=int(capacity_str)
        )

    def parse(self, filepath: str) -> tuple[list[Hub], list[Connection], int]:
        list_of_nodes: list[Hub] = []
        list_of_conection: list[Connection] = []
        nb_drones = 0
        seen_hubs = set()
        seen_connections = set()
        stup_check_start = False
        stup_check_end = False
        with open(filepath, "r") as file:
            for line in file:
                clean_line = line.split("#")[0].strip()
                if not clean_line:
                    continue
                if clean_line.startswith("nb_drones:"):
                    nb_drones = int(clean_line.split(":", 1)[1].strip())
                elif clean_line.startswith("start_hub:"):
                    if stup_check_start:
                        raise ValueError("error, we already had start hub!")
                    stup_check_start = True
                    hub_data = clean_line.split(":", 1)[1].strip()
                    new_hub = self._parse_hub_line(
                        hub_data, is_start=True)
                    if new_hub.name in seen_hubs:
                        raise ValueError(
                            f"Parse Error: Duplicate hub name "
                            f"'{new_hub.name}' detected.")
                    seen_hubs.add(new_hub.name)
                    list_of_nodes.append(new_hub)
                elif clean_line.startswith("hub:"):
                    hub_data = clean_line.split(":", 1)[1].strip()
                    new_hub = self._parse_hub_line(hub_data)
                    if new_hub.name in seen_hubs:
                        raise ValueError(
                            f"Parse Error: Duplicate hub name "
                            f"'{new_hub.name}' detected.")
                    seen_hubs.add(new_hub.name)
                    list_of_nodes.append(new_hub)
                elif clean_line.startswith("end_hub:"):
                    if stup_check_end:
                        raise ValueError("error, we already had end hub!")
                    stup_check_end = True
                    hub_data = clean_line.split(":", 1)[1].strip()
                    new_hub = self._parse_hub_line(
                        hub_data, is_end=True)
                    if new_hub.name in seen_hubs:
                        raise ValueError(
                            f"Parse Error: Duplicate hub name "
                            f"'{new_hub.name}' detected.")
                    seen_hubs.add(new_hub.name)
                    list_of_nodes.append(new_hub)
                elif clean_line.startswith("connection"):
                    connection_data = clean_line.split(":", 1)[1].strip()
                    new_connection = self._conection_lines_sep(
                        connection_data
                    )
                    connection_pair = tuple(
                        sorted(
                            [new_connection.from_hub, new_connection.to_hub]))
                    if connection_pair in seen_connections:
                        raise ValueError(
                            f"Parse Error: Duplicate connection between "
                            f"{connection_pair[0]} and {connection_pair[1]}")
                    seen_connections.add(connection_pair)
                    list_of_conection.append(new_connection)

        return list_of_nodes, list_of_conection, nb_drones
