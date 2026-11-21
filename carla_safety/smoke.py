"""Minimal connectivity smoke test: server/client versions and map/actors."""
import sys

import carla


def main() -> int:
    host = sys.argv[1] if len(sys.argv) > 1 else "127.0.0.1"
    port = int(sys.argv[2]) if len(sys.argv) > 2 else 2000

    client = carla.Client(host, port)
    client.set_timeout(60.0)
    world = client.get_world()

    print("CARLA client version:", getattr(carla, "__version__", "0.9.16"))
    print("CARLA server version:", client.get_server_version())
    print("map:", world.get_map().name)
    print("actors:", len(world.get_actors()))
    print("SMOKE OK")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
