class SimulationPrinter:
    @staticmethod
    def print_turn_by_turn(all_drone_paths: dict[int, list[str]]) -> None:
        """Converts space-time paths into turn-by-turn drone move commands."""
        # Find the maximum total turns taken by any drone
        max_turns = max(len(path) for path in all_drone_paths.values())

        print("\n=== TURN-BY-TURN SIMULATION ===")
        
        for t in range(1, max_turns):
            turn_moves: list[str] = []
            
            for drone_id, path in all_drone_paths.items():
                # If the drone has a step at tick t and moved to a new hub:
                if t < len(path):
                    prev_hub = path[t - 1]
                    curr_hub = path[t]
                    
                    # Only print if the drone actually moved (didn't wait in place)
                    if curr_hub != prev_hub:
                        turn_moves.append(f"D{drone_id}-{curr_hub}")

            if turn_moves:
                print(f"Turn {t:2d}: " + " ".join(turn_moves))