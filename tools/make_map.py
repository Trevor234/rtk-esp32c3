#!/usr/bin/env python3
"""Download OSM tiles covering a small area and pack them into the raw
container the firmware's mapdata partition expects.

Default coverage: Demonstration coordinates +/- 1.2 km
(zoom 18, ~360 tiles, palette-PNG compressed to fit the ~2.4 MB partition).

The container layout must match firmware/main/map_store.c exactly:
  header  32 B : magic "USVM", version u16, zoom u8, res u8, count u16,
                 res u16, min_x u32, max_x u32, min_y u32, max_y u32,
                 index_offset u32 (== 32)
  index   count * 16 B : { x u32, y u32, data offset u32, length u32 }
                 sorted by (x, y) - the firmware binary-searches it
  data    concatenated palette PNG tiles

Tile usage follows the OSM tile usage policy: a descriptive User-Agent, no
parallel downloads, caching, and back-off on 429/503.  Bulk mirror downloads
are NOT what this tool is for - keep the area small.

Usage:
  python tools/make_map.py                       # default coverage
  python tools/make_map.py --radius-m 600 --zoom 19   # smaller, sharper core
  python tools/make_map.py --no-recompress       # keep original OSM PNGs

Then flash (never run erase_flash, it would wipe the map again):
  esptool --chip esp32c3 -p PORT write_flash 0x187000 tools/map.bin
"""

import argparse
import math
import os
import struct
import sys
import time
import urllib.request
import urllib.error

MAGIC = 0x4D565355          # "USVM" little-endian, matches map_store.c
VERSION = 1
HDR_FMT = "<IHBBHHIIIII"    # magic, ver, zoom, res, count, res, 4 bounds, idx_off = 32 B
HDR_SIZE = struct.calcsize(HDR_FMT)
IDX_FMT = "<IIII"
IDX_SIZE = struct.calcsize(IDX_FMT)

MAP_PARTITION_OFFSET = 0x187000
MAP_PARTITION_SIZE = 0x278000

# Default center (demonstration coordinates)
DEFAULT_CENTER = (31.0000, 121.0000)

USER_AGENT = "gps-sim/1.0 (one-off small tile batch)"
DEFAULT_URL = "https://tile.openstreetmap.org/{z}/{x}/{y}.png"


def deg2tilexy(lat, lon, z):
    n = 1 << z
    lat = max(-85.05112878, min(85.05112878, lat))
    xt = (lon + 180.0) / 360.0 * n
    lr = math.radians(lat)
    yt = (1.0 - math.log(math.tan(lr) + 1.0 / math.cos(lr)) / math.pi) / 2.0 * n
    return xt, yt


def tile_range(lat, lon, radius_m, z):
    lat_span = radius_m / 111132.0
    lon_span = radius_m / (111320.0 * math.cos(math.radians(lat)))
    x_min, y_min = deg2tilexy(lat + lat_span, lon - lon_span, z)
    x_max, y_max = deg2tilexy(lat - lat_span, lon + lon_span, z)
    xs = range(int(math.floor(x_min)), int(math.floor(x_max)) + 1)
    ys = range(int(math.floor(y_min)), int(math.floor(y_max)) + 1)
    return xs, ys


def fetch(url, cache_path):
    """Download one tile with a cache; retries politely on 429/503."""
    if os.path.exists(cache_path) and os.path.getsize(cache_path) > 0:
        with open(cache_path, "rb") as f:
            return f.read()
    delay = 1.0
    for attempt in range(4):
        try:
            req = urllib.request.Request(url, headers={"User-Agent": USER_AGENT})
            with urllib.request.urlopen(req, timeout=20) as r:
                data = r.read()
            tmp = cache_path + ".tmp"
            with open(tmp, "wb") as f:
                f.write(data)
            os.replace(tmp, cache_path)
            return data
        except urllib.error.HTTPError as e:
            if e.code in (429, 503):
                print(f"    {e.code}, backing off {delay:.0f}s")
                time.sleep(delay)
                delay *= 2
                continue
            raise
        except urllib.error.URLError as e:
            # transient TLS/connection resets happen on this route; back off
            print(f"    network error ({e.reason}), backing off {delay:.0f}s")
            time.sleep(delay)
            delay *= 2
            continue
    raise RuntimeError(f"giving up on {url}")


def recompress(raw, colors):
    """Palette-compress a tile PNG; falls back to RGB PNG if PIL is absent."""
    try:
        from io import BytesIO
        from PIL import Image
        img = Image.open(BytesIO(raw)).convert("RGB")
        out = BytesIO()
        img.quantize(colors=colors, method=Image.MEDIANCUT).save(
            out, format="PNG", optimize=True)
        return out.getvalue()
    except ImportError:
        return raw


def build(tiles, zoom, budget, colors_seq, recompress_on):
    """tiles: dict {(x, y): raw png}.  Try palette sizes until it fits."""
    for colors in colors_seq:
        entries, blob, offset = [], bytearray(), HDR_SIZE + len(tiles) * IDX_SIZE
        for (x, y) in sorted(tiles):
            body = recompress(tiles[(x, y)], colors) if recompress_on else tiles[(x, y)]
            if len(body) > 32768:
                raise RuntimeError(f"tile {x},{y} too large ({len(body)} B)")
            entries.append((x, y, offset, len(body)))
            blob += body
            offset += len(body)
        total = HDR_SIZE + len(tiles) * IDX_SIZE + len(blob)
        xs = [e[0] for e in entries]
        ys = [e[1] for e in entries]
        hdr = struct.pack(HDR_FMT, MAGIC, VERSION, zoom, 0, len(tiles), 0,
                          min(xs), max(xs), min(ys), max(ys), HDR_SIZE)
        idx = b"".join(struct.pack(IDX_FMT, *e) for e in entries)
        if total <= budget:
            print(f"  palette {colors} colors: total {total} B ({total/1024:.0f} KB) - fits")
            return hdr + idx + bytes(blob), total
        print(f"  palette {colors} colors: total {total} B - over budget, trying fewer")
    raise RuntimeError(f"cannot fit under {budget} B even at {colors_seq[-1]} colors")


def main():
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--center", nargs=2, type=float, default=DEFAULT_CENTER,
                    metavar=("LAT", "LON"), help="area center, WGS84 degrees")
    ap.add_argument("--radius-m", type=float, default=1200.0,
                    help="coverage radius in metres (default 1200)")
    ap.add_argument("--zoom", type=int, default=18, help="tile zoom (default 18)")
    ap.add_argument("--budget", type=int, default=MAP_PARTITION_SIZE - 16000,
                    help="max container bytes (default partition minus margin)")
    ap.add_argument("--url", default=DEFAULT_URL, help="tile URL template")
    ap.add_argument("--out", default=os.path.join(os.path.dirname(__file__), "map.bin"))
    ap.add_argument("--cache-dir", default=os.path.join(os.path.dirname(__file__), "tilecache"))
    ap.add_argument("--colors", type=int, nargs="+", default=[64, 48, 32, 24, 16],
                    help="adaptive palette sizes, tried in order")
    ap.add_argument("--no-recompress", action="store_true",
                    help="keep the original OSM PNGs untouched")
    args = ap.parse_args()

    lat, lon = args.center
    xs, ys = tile_range(lat, lon, args.radius_m, args.zoom)
    n_tiles = len(list(xs)) * len(list(ys))
    print(f"center {lat:.5f},{lon:.5f}  radius {args.radius_m:.0f} m  "
          f"zoom {args.zoom}  tiles {n_tiles}  budget {args.budget} B")

    os.makedirs(args.cache_dir, exist_ok=True)
    tiles, done = {}, 0
    t0 = time.time()
    for x in xs:
        for y in ys:
            url = args.url.format(z=args.zoom, x=x, y=y)
            raw = fetch(url, os.path.join(args.cache_dir, f"{args.zoom}_{x}_{y}.png"))
            tiles[(x, y)] = raw
            done += 1
            if done % 25 == 0:
                print(f"  downloaded {done}/{n_tiles} ({time.time()-t0:.0f}s)")
            time.sleep(0.12)          # OSM policy: no parallel bulk fetching
    print(f"downloaded {len(tiles)} tiles in {time.time()-t0:.0f}s")

    if args.zoom > 19:
        print("warning: zooms above 19 are rarely rendered upstream")
    data, total = build(tiles, args.zoom, args.budget, args.colors,
                        not args.no_recompress)
    with open(args.out, "wb") as f:
        f.write(data)
    print(f"wrote {args.out} ({total} B)")
    if total > MAP_PARTITION_SIZE:
        print(f"ERROR: larger than the mapdata partition ({MAP_PARTITION_SIZE} B)")
        sys.exit(1)
    print(f"flash with:\n  esptool --chip esp32c3 -p PORT write_flash "
          f"{MAP_PARTITION_OFFSET:#x} {args.out}")


if __name__ == "__main__":
    main()
