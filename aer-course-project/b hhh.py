import itertools

gate_coordinates = [
    [0.5, -2.5, 0, 0, 0, -1.57, 0],  # gate 1
    [2.0, -1.5, 0, 0, 0, 0, 0],      # gate 2
    [0.0, 0.2, 0, 0, 0, 1.57, 0],    # gate 3
    [-0.5, 1.5, 0, 0, 0, 0, 0]       # gate 4
]

# Generate all permutations of the gate order
permutations = itertools.permutations(gate_coordinates)

# Print all combinations
for idx, perm in enumerate(permutations, start=1):
    print(f"Combination {idx}:")
    for gate_idx, gate_coords in enumerate(perm, start=1):
        print(f"{gate_coords},")
    print()
