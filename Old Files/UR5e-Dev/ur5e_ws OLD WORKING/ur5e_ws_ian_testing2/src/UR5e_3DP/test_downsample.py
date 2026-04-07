import math

def gcode_to_robot(gx, gy, gz):
    # Dummy implementation for testing distance
    return gx, gy, gz

def downsample(pts, min_dist_m):
    if not pts:
        return pts

    kept  = [pts[0].copy()]
    acc_e = pts[0]['e_delta']

    for p in pts[1:]:
        prev = kept[-1]

        # ── Type boundary: flush and keep ────────────────────────────────────
        if p['is_travel'] != prev['is_travel']:
            # Flush any accumulated extrusion into the last kept point
            kept[-1]['e_delta'] = acc_e
            kept.append(p.copy())
            acc_e = p['e_delta']
            continue

        # ── Travel move: always keep ──────────────────────────────────────────
        if p['is_travel']:
            kept[-1]['e_delta'] = acc_e
            kept.append(p.copy())
            acc_e = p['e_delta']
            continue

        # ── Extrusion move: apply distance filter ────────────────────────────
        rx_prev, ry_prev, _ = gcode_to_robot(prev['x'], prev['y'], prev['z'])
        rx_cur,  ry_cur,  _ = gcode_to_robot(p['x'],   p['y'],   p['z'])
        d = math.sqrt((rx_cur - rx_prev)**2 + (ry_cur - ry_prev)**2)
        acc_e += p['e_delta']

        if d >= min_dist_m:
            p_copy = p.copy()
            p_copy['e_delta'] = acc_e
            kept.append(p_copy)
            acc_e = 0.0

    # Flush any remaining accumulated extrusion into the last kept point
    kept[-1]['e_delta'] = kept[-1]['e_delta'] + acc_e if acc_e > 0 else kept[-1]['e_delta']

    return kept

def test():
    # Sequence of points
    pts = [
        {'x': 0, 'y': 0, 'z': 0, 'e_delta': 0.0, 'is_travel': True},   # travel
        {'x': 1, 'y': 0, 'z': 0, 'e_delta': 1.0, 'is_travel': False},  # extrude 1
        {'x': 2, 'y': 0, 'z': 0, 'e_delta': 1.0, 'is_travel': False},  # extrude 2
        {'x': 3, 'y': 0, 'z': 0, 'e_delta': 1.0, 'is_travel': False},  # extrude 3
        {'x': 4, 'y': 0, 'z': 0, 'e_delta': 1.0, 'is_travel': False},  # extrude 4
        {'x': 5, 'y': 5, 'z': 0, 'e_delta': 0.0, 'is_travel': True},   # travel
    ]
    
    # Let min_dist be larger than 1 to test combination, e.g. 2.5
    filtered = downsample(pts, min_dist_m=2.5)
    
    print("Original points:")
    for i, p in enumerate(pts):
        print(f"[{i}] {p}")
        
    print("\nDownsampled points (min_dist=2.5):")
    for i, p in enumerate(filtered):
        print(f"[{i}] {p}")
        
    print("\nTotal extrusion original:", sum(p['e_delta'] for p in pts))
    print("Total extrusion downsampled:", sum(p['e_delta'] for p in filtered))

    # Test with min_dist 0 so nothing gets merged
    filtered_0 = downsample(pts, min_dist_m=0.1)
    # print("\nDownsampled points (min_dist=0.1):")
    # for i, p in enumerate(filtered_0):
    #     print(f"[{i}] {p}")
    print("Total extrusion downsampled (0.1):", sum(p['e_delta'] for p in filtered_0))

if __name__ == '__main__':
    test()
