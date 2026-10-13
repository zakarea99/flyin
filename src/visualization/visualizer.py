import sys
import pygame
from core.graph1 import Graph
from src.model.ZoneType import ZoneType

# --- Color Palette (Catppuccin Dark Theme) ---
BG_COLOR = (24, 24, 37)
LINE_COLOR = (88, 91, 112)
TEXT_COLOR = (205, 214, 244)
PANEL_BG = (30, 30, 46)

# Zone & Hub Colors
COLOR_START = (166, 227, 161)      # Soft Green
COLOR_GOAL = (243, 139, 168)       # Red / Pink
COLOR_RESTRICTED = (250, 179, 135) # Orange
COLOR_BLOCKED = (108, 112, 134)    # Dark Slate
COLOR_NORMAL = (137, 180, 250)     # Light Blue

# Drone Colors (Palette cycling for distinct drone IDs)
DRONE_PALETTE = [
    (245, 194, 231), (203, 166, 247), (148, 226, 213),
    (249, 226, 175), (235, 160, 172), (180, 190, 254),
    (114, 241, 184), (250, 204, 21)
]


class PygameVisualizer:
    def __init__(
        self,
        graph: Graph,
        all_drone_paths: dict[int, list[str]],
        width: int = 1200,
        height: int = 800
    ):
        self.graph = graph
        self.paths = all_drone_paths
        self.width = width
        self.height = height

        # Simulation parameters
        self.max_ticks = max(len(p) for p in self.paths.values()) if self.paths else 0
        self.current_tick = 0
        self.progress = 0.0          # 0.0 to 1.0 interpolation between ticks
        self.speed = 1.5             # Ticks per second
        self.paused = False

        # Pygame setup
        pygame.init()
        pygame.font.init()
        self.screen = pygame.display.set_mode((self.width, self.height))
        pygame.display.set_caption("Fly-in Drone Routing Engine - Space-Time Visualizer")
        self.clock = pygame.time.Clock()
        
        self.font = pygame.font.SysFont("dejavusans", 12, bold=True)
        self.hud_font = pygame.font.SysFont("dejavusans", 16, bold=True)

        # Precompute normalized screen positions for each hub
        self.screen_coords: dict[str, tuple[float, float]] = self._calculate_screen_coords()

    def _calculate_screen_coords(self) -> dict[str, tuple[float, float]]:
        """Scales graph (x,y) coordinates to fit comfortably within window padding."""
        margin = 100
        usable_w = self.width - 2 * margin
        usable_h = self.height - 2 * margin

        xs = [h.x for h in self.graph.hubs.values()]
        ys = [h.y for h in self.graph.hubs.values()]

        min_x, max_x = min(xs), max(xs)
        min_y, max_y = min(ys), max(ys)

        span_x = (max_x - min_x) if max_x != min_x else 1.0
        span_y = (max_y - min_y) if max_y != min_y else 1.0

        coords = {}
        for name, hub in self.graph.hubs.items():
            norm_x = (hub.x - min_x) / span_x
            norm_y = (hub.y - min_y) / span_y
            
            # Map with inverted Y for screen space
            sx = margin + norm_x * usable_w
            sy = self.height - (margin + norm_y * usable_h)
            coords[name] = (sx, sy)

        return coords

    def run(self) -> None:
        """Main Pygame execution loop."""
        running = True
        while running:
            dt = self.clock.tick(60) / 1000.0  # Delta time in seconds

            # 1. Process Input Events
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    running = False
                elif event.type == pygame.KEYDOWN:
                    if event.key == pygame.K_SPACE:
                        self.paused = not self.paused
                    elif event.key == pygame.K_RIGHT:
                        self.current_tick = min(self.max_ticks - 1, self.current_tick + 1)
                        self.progress = 0.0
                    elif event.key == pygame.K_LEFT:
                        self.current_tick = max(0, self.current_tick - 1)
                        self.progress = 0.0
                    elif event.key == pygame.K_UP:
                        self.speed = min(10.0, self.speed + 0.5)
                    elif event.key == pygame.K_DOWN:
                        self.speed = max(0.5, self.speed - 0.5)

            # 2. Update Simulation Progress
            if not self.paused and self.max_ticks > 0:
                self.progress += dt * self.speed
                if self.progress >= 1.0:
                    steps = int(self.progress)
                    self.progress -= steps
                    self.current_tick += steps
                    if self.current_tick >= self.max_ticks - 1:
                        self.current_tick = self.max_ticks - 1
                        self.progress = 0.0
                        self.paused = True

            # 3. Render
            self.screen.fill(BG_COLOR)
            self._draw_connections()
            self._draw_hubs()
            self._draw_drones()
            self._draw_hud()

            pygame.display.flip()

        pygame.quit()

    def _draw_connections(self) -> None:
        """Draws network connections (links) between hubs."""
        drawn_links = set()
        for u, neighbors in self.graph.adjacency_list.items():
            for v in neighbors:
                link_key = tuple(sorted([u, v]))
                if link_key in drawn_links:
                    continue
                drawn_links.add(link_key)

                p1 = self.screen_coords[u]
                p2 = self.screen_coords[v]
                pygame.draw.line(self.screen, LINE_COLOR, p1, p2, width=2)

    def _draw_hubs(self) -> None:
        """Draws hub nodes with zone styling and label names."""
        for name, hub in self.graph.hubs.items():
            pos = (int(self.screen_coords[name][0]), int(self.screen_coords[name][1]))

            # Determine Node Color
            if hub.is_start:
                color = COLOR_START
                radius = 16
            elif hub.is_end:
                color = COLOR_GOAL
                radius = 16
            elif hub.zone_type == ZoneType.blocked:
                color = COLOR_BLOCKED
                radius = 10
            elif hub.zone_type == ZoneType.restricted:
                color = COLOR_RESTRICTED
                radius = 12
            else:
                color = COLOR_NORMAL
                radius = 12

            # Draw hub node & border
            pygame.draw.circle(self.screen, color, pos, radius)
            pygame.draw.circle(self.screen, (255, 255, 255), pos, radius, width=1)

            # Draw Hub Label
            text_surf = self.font.render(name, True, TEXT_COLOR)
            self.screen.blit(text_surf, (pos[0] - text_surf.get_width() // 2, pos[1] + radius + 4))

    def _draw_drones(self) -> None:
        """Draws drones with smooth interpolation between turn steps."""
        for drone_id, path in self.paths.items():
            if not path:
                continue

            # Determine source and target positions for current interpolation
            idx_from = min(self.current_tick, len(path) - 1)
            idx_to = min(self.current_tick + 1, len(path) - 1)

            hub_from = path[idx_from]
            hub_to = path[idx_to]

            p1 = self.screen_coords[hub_from]
            p2 = self.screen_coords[hub_to]

            # Linear Interpolation (Lerp)
            cur_x = p1[0] + (p2[0] - p1[0]) * self.progress
            cur_y = p1[1] + (p2[1] - p1[1]) * self.progress
            drone_pos = (int(cur_x), int(cur_y))

            # Select unique color per drone
            color = DRONE_PALETTE[(drone_id - 1) % len(DRONE_PALETTE)]

            # Draw Drone Marker
            pygame.draw.circle(self.screen, color, drone_pos, 10)
            pygame.draw.circle(self.screen, (0, 0, 0), drone_pos, 10, width=1)

            # Draw Drone ID Label
            id_text = self.hud_font.render(f"D{drone_id}", True, (0, 0, 0))
            self.screen.blit(id_text, (drone_pos[0] - id_text.get_width() // 2, drone_pos[1] - id_text.get_height() // 2))

    def _draw_hud(self) -> None:
        """Draws heads-up display overlay for turn status and controls."""
        panel_rect = pygame.Rect(15, 15, 320, 100)
        pygame.draw.rect(self.screen, PANEL_BG, panel_rect, border_radius=8)
        pygame.draw.rect(self.screen, LINE_COLOR, panel_rect, width=1, border_radius=8)

        status = "PAUSED" if self.paused else "RUNNING"
        line1 = f"Turn: {self.current_tick + 1} / {self.max_ticks}  [{status}]"
        line2 = f"Speed: {self.speed:.1f}x (UP/DOWN)"
        line3 = "SPACE: Pause/Play  |  LEFT/RIGHT: Step"

        t1 = self.hud_font.render(line1, True, TEXT_COLOR)
        t2 = self.font.render(line2, True, TEXT_COLOR)
        t3 = self.font.render(line3, True, COLOR_RESTRICTED)

        self.screen.blit(t1, (25, 25))
        self.screen.blit(t2, (25, 55))
        self.screen.blit(t3, (25, 80))