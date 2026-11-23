"""
Intelligent Courier Planning & Route Optimization System
Capstone Project - Integrated Algorithmic Prototype

Integrates:
1. Merge Sort (Parcel Prioritization by Priority, Deadline & Value Density)
2. 0/1 Knapsack Dynamic Programming (Optimal Vehicle Box Filling)
3. Dijkstra's Shortest Path Algorithm (Customer Road Network Distance Matrix)
4. Nearest-Neighbor TSP Approximation (Ordered Delivery Route Construction)
5. Multi-Scale Performance Benchmarking & Visualization
"""

import json
import time
import random
import heapq
from dataclasses import dataclass, asdict
from typing import List, Dict, Tuple, Optional


@dataclass
class Parcel:
    parcel_id: str
    destination: int
    weight: int
    value: int
    deadline_hrs: int
    priority: int  # 1 = Urgent, 2 = Express, 3 = Standard


# ==============================================================================
# PHASE 2A: PACKAGE PRIORITIZATION USING MERGE SORT — O(P log P)
# ==============================================================================
def parcel_sort_key(p: Parcel) -> Tuple[int, int, float]:
    """Sort by Priority (asc), Deadline (asc), and Value-to-Weight Ratio (desc)."""
    density = p.value / max(p.weight, 1)
    return (p.priority, p.deadline_hrs, -density)


def merge_sort_parcels(parcels: List[Parcel]) -> List[Parcel]:
    """Deterministic Merge Sort implementation for parcel prioritization."""
    if len(parcels) <= 1:
        return parcels[:]

    mid = len(parcels) // 2
    left = merge_sort_parcels(parcels[:mid])
    right = merge_sort_parcels(parcels[mid:])

    merged = []
    i = j = 0
    while i < len(left) and j < len(right):
        if parcel_sort_key(left[i]) <= parcel_sort_key(right[j]):
            merged.append(left[i])
            i += 1
        else:
            merged.append(right[j])
            j += 1

    merged.extend(left[i:])
    merged.extend(right[j:])
    return merged


# ==============================================================================
# PHASE 2B: OPTIMAL BOX FILLING USING 0/1 KNAPSACK DP — O(P * W)
# ==============================================================================
def knapsack_box_filling(parcels: List[Parcel], capacity: int) -> Tuple[List[Parcel], int, int, List[List[int]]]:
    """
    Solves 0/1 Knapsack using bottom-up Dynamic Programming.
    Returns: (selected_parcels, max_value, total_weight, dp_table)
    """
    n = len(parcels)
    dp = [[0] * (capacity + 1) for _ in range(n + 1)]

    for i in range(1, n + 1):
        w_i = parcels[i - 1].weight
        v_i = parcels[i - 1].value
        for w in range(capacity + 1):
            if w_i <= w:
                dp[i][w] = max(dp[i - 1][w], dp[i - 1][w - w_i] + v_i)
            else:
                dp[i][w] = dp[i - 1][w]

    # Backtrack through DP table to recover exact selected parcels
    selected = []
    w = capacity
    for i in range(n, 0, -1):
        if dp[i][w] != dp[i - 1][w]:
            selected.append(parcels[i - 1])
            w -= parcels[i - 1].weight

    selected.reverse()
    total_weight = sum(p.weight for p in selected)
    max_value = dp[n][capacity]
    return selected, max_value, total_weight, dp


# ==============================================================================
# PHASE 3A: GRAPH REPRESENTATION & DIJKSTRA'S SHORTEST PATH — O(V^2) / O((V+E)logV)
# ==============================================================================
class DeliveryGraph:
    def __init__(self, num_vertices: int):
        self.V = num_vertices
        self.adj: Dict[int, List[Tuple[int, int]]] = {i: [] for i in range(num_vertices)}

    def add_edge(self, u: int, v: int, weight: int) -> None:
        self.adj[u].append((v, weight))
        self.adj[v].append((u, weight))

    def dijkstra(self, source: int) -> Tuple[List[float], List[Optional[int]]]:
        """Standard Dijkstra's algorithm returning shortest distances and parent pointers."""
        INF = float("inf")
        dist = [INF] * self.V
        parent: List[Optional[int]] = [None] * self.V
        visited = [False] * self.V
        dist[source] = 0

        for _ in range(self.V):
            u = -1
            best = INF
            for v in range(self.V):
                if not visited[v] and dist[v] < best:
                    best = dist[v]
                    u = v
            if u == -1:
                break
            visited[u] = True
            for neighbor, w in self.adj[u]:
                if not visited[neighbor] and dist[u] + w < dist[neighbor]:
                    dist[neighbor] = dist[u] + w
                    parent[neighbor] = u

        return dist, parent

    def dijkstra_heap(self, source: int) -> Tuple[List[float], List[Optional[int]]]:
        """Optimized Min-Heap Dijkstra O((V + E) log V) for larger graphs."""
        INF = float("inf")
        dist = [INF] * self.V
        parent: List[Optional[int]] = [None] * self.V
        dist[source] = 0
        pq = [(0, source)]

        while pq:
            d, u = heapq.heappop(pq)
            if d > dist[u]:
                continue
            for neighbor, w in self.adj[u]:
                if dist[u] + w < dist[neighbor]:
                    dist[neighbor] = dist[u] + w
                    parent[neighbor] = u
                    heapq.heappush(pq, (dist[neighbor], neighbor))

        return dist, parent

    @staticmethod
    def reconstruct_path(parent: List[Optional[int]], target: int) -> List[int]:
        path = []
        curr: Optional[int] = target
        while curr is not None:
            path.append(curr)
            curr = parent[curr]
        path.reverse()
        return path


# ==============================================================================
# PHASE 3B: NEAREST-NEIGHBOR TSP APPROXIMATION — O(K^2)
# ==============================================================================
def nearest_neighbor_tsp(
    depot: int,
    stops: List[int],
    dist_matrix: Dict[int, List[float]],
    parent_matrix: Dict[int, List[Optional[int]]],
) -> Tuple[List[int], List[int], float, List[str]]:
    """
    Constructs an approximate TSP tour starting and ending at `depot`,
    visiting all unique customer `stops` using the Nearest-Neighbor heuristic.
    """
    unvisited = sorted(set(s for s in stops if s != depot))
    tour_stops = [depot]
    full_road_path = [depot]
    total_distance = 0.0
    step_logs = []

    current = depot
    while unvisited:
        next_stop = min(unvisited, key=lambda node: (dist_matrix[current][node], node))
        leg_dist = dist_matrix[current][next_stop]
        if leg_dist == float("inf"):
            raise ValueError(f"Destination {next_stop} is unreachable from {current}.")

        leg_path = DeliveryGraph.reconstruct_path(parent_matrix[current], next_stop)
        full_road_path.extend(leg_path[1:])
        total_distance += leg_dist
        step_logs.append(
            f"Stop {current} -> Stop {next_stop} | Distance: {int(leg_dist)} km | Road Path: {' -> '.join(map(str, leg_path))}"
        )
        tour_stops.append(next_stop)
        unvisited.remove(next_stop)
        current = next_stop

    # Return leg back to depot
    return_dist = dist_matrix[current][depot]
    return_path = DeliveryGraph.reconstruct_path(parent_matrix[current], depot)
    full_road_path.extend(return_path[1:])
    total_distance += return_dist
    tour_stops.append(depot)
    step_logs.append(
        f"Stop {current} -> Hub {depot}  | Distance: {int(return_dist)} km | Road Path: {' -> '.join(map(str, return_path))}"
    )

    return tour_stops, full_road_path, total_distance, step_logs


# ==============================================================================
# END-TO-END PIPELINE & BENCHMARKING
# ==============================================================================
def run_courier_pipeline(
    parcels: List[Parcel],
    capacity: int,
    graph: DeliveryGraph,
    depot: int = 0,
    verbose: bool = True,
) -> Dict:
    timings = {}

    # Step 1: Merge Sort Prioritization
    t0 = time.perf_counter()
    sorted_parcels = merge_sort_parcels(parcels)
    timings["merge_sort_ms"] = (time.perf_counter() - t0) * 1000.0

    # Step 2: 0/1 Knapsack DP Optimization
    t0 = time.perf_counter()
    selected_parcels, max_val, used_weight, dp_table = knapsack_box_filling(sorted_parcels, capacity)
    timings["knapsack_dp_ms"] = (time.perf_counter() - t0) * 1000.0

    # Step 3: Dijkstra Shortest Paths for Active Nodes
    active_stops = sorted({depot} | {p.destination for p in selected_parcels})
    t0 = time.perf_counter()
    dist_matrix = {}
    parent_matrix = {}
    for node in active_stops:
        d, p = graph.dijkstra(node)
        dist_matrix[node] = d
        parent_matrix[node] = p
    timings["dijkstra_ms"] = (time.perf_counter() - t0) * 1000.0

    # Step 4: Nearest-Neighbor TSP Route Planning
    t0 = time.perf_counter()
    tour_stops, full_road_path, total_route_dist, step_logs = nearest_neighbor_tsp(
        depot, active_stops, dist_matrix, parent_matrix
    )
    timings["tsp_nn_ms"] = (time.perf_counter() - t0) * 1000.0
    timings["total_ms"] = sum(timings.values())

    if verbose:
        print("=" * 78)
        print("INTELLIGENT COURIER PLANNING & ROUTE OPTIMIZATION SYSTEM")
        print("=" * 78)
        print(f"Total Parcels Received : {len(parcels)} | Vehicle Capacity : {capacity} kg")
        print(f"Road Network Topology  : {graph.V} Locations (Hub = {depot})")
        print("-" * 78)

        print("\n[PHASE 2A] PRIORITIZED PARCELS (MERGE SORT OUTPUT)")
        print(f"{'ID':<8} {'Dest':<6} {'Wt(kg)':<8} {'Value':<8} {'Deadline':<10} {'Priority':<10}")
        for p in sorted_parcels:
            p_label = {1: "1 (Urgent)", 2: "2 (Express)", 3: "3 (Normal)"}[p.priority]
            print(f"{p.parcel_id:<8} {p.destination:<6} {p.weight:<8} {p.value:<8} {p.deadline_hrs:<10} {p_label:<10}")

        print("\n[PHASE 2B] OPTIMAL PACKING PLAN (0/1 KNAPSACK DP OUTPUT)")
        print(f"Selected Parcels : {', '.join(p.parcel_id for p in selected_parcels)}")
        print(f"Weight Utilized  : {used_weight} / {capacity} kg ({(used_weight / capacity) * 100:.1f}% full)")
        print(f"Total Value Score: {max_val}")

        print("\nDP Table Snapshot (Rows = First k Parcels, Cols = Capacity 0..W):")
        header = "Parcel\\Cap | " + " ".join(f"{w:>3}" for w in range(capacity + 1))
        print(header)
        print("-" * len(header))
        for i in range(len(sorted_parcels) + 1):
            label = "None" if i == 0 else sorted_parcels[i - 1].parcel_id
            row_str = " ".join(f"{dp_table[i][w]:>3}" for w in range(capacity + 1))
            print(f"{label:<10} | {row_str}")

        print("\n[PHASE 3A] DIJKSTRA SHORTEST PATHS FROM CENTRAL HUB (NODE 0)")
        hub_dist, hub_parent = dist_matrix[depot], parent_matrix[depot]
        for v in range(graph.V):
            path_str = " -> ".join(map(str, DeliveryGraph.reconstruct_path(hub_parent, v)))
            print(f"Hub 0 -> Node {v} : Distance = {int(hub_dist[v]):>2} km | Shortest Path: {path_str}")

        print("\n[PHASE 3B] ORDERED DELIVERY ROUTE (NEAREST-NEIGHBOR TSP APPROXIMATION)")
        for log in step_logs:
            print("  " + log)
        print(f"\nCustomer Visit Sequence : {' -> '.join(map(str, tour_stops))}")
        print(f"Complete Road Traversal : {' -> '.join(map(str, full_road_path))}")
        print(f"Total Route Length      : {int(total_route_dist)} km")
        print("=" * 78)

    return {
        "sorted_parcels": sorted_parcels,
        "selected_parcels": selected_parcels,
        "used_weight": used_weight,
        "max_value": max_val,
        "dp_table": dp_table,
        "tour_stops": tour_stops,
        "full_road_path": full_road_path,
        "total_route_dist": total_route_dist,
        "timings": timings,
    }


def generate_synthetic_dataset(num_parcels: int, capacity: int, num_vertices: int, seed: int = 42):
    rng = random.Random(seed)
    parcels = [
        Parcel(
            parcel_id=f"PKG-{i+1:03d}",
            destination=rng.randint(1, num_vertices - 1),
            weight=rng.randint(2, 12),
            value=rng.randint(15, 100),
            deadline_hrs=rng.choice([2, 4, 6, 8, 12, 24]),
            priority=rng.choice([1, 2, 3]),
        )
        for i in range(num_parcels)
    ]

    graph = DeliveryGraph(num_vertices)
    edges = []
    for v in range(1, num_vertices):
        u = rng.randint(0, v - 1)
        w = rng.randint(2, 15)
        graph.add_edge(u, v, w)
        edges.append((u, v, w))

    existing = {(min(u, v), max(u, v)) for u, v, _ in edges}
    target_edges = min(num_vertices * (num_vertices - 1) // 2, int(num_vertices * 2.5))
    while len(edges) < target_edges:
        u = rng.randint(0, num_vertices - 1)
        v = rng.randint(0, num_vertices - 1)
        if u != v and (min(u, v), max(u, v)) not in existing:
            w = rng.randint(2, 20)
            graph.add_edge(u, v, w)
            edges.append((u, v, w))
            existing.add((min(u, v), max(u, v)))

    return parcels, capacity, graph, edges


def run_benchmarks():
    scales = [
        ("Small (P=8, W=15, V=6)", 8, 15, 6),
        ("Medium (P=50, W=80, V=25)", 50, 80, 25),
        ("Large (P=200, W=250, V=60)", 200, 250, 60),
        ("Stress (P=500, W=500, V=120)", 500, 500, 120),
    ]
    print("\n[PHASE 4] MULTI-SCALE RUNTIME BENCHMARK (MILLISECONDS)")
    print(f"{'Dataset Scale':<30} | {'MergeSort':>10} | {'Knapsack DP':>12} | {'Dijkstra':>10} | {'TSP (NN)':>10} | {'Total (ms)':>11}")
    print("-" * 96)
    for label, p_cnt, cap, v_cnt in scales:
        ps, c, g, _ = generate_synthetic_dataset(p_cnt, cap, v_cnt, seed=42)
        res = run_courier_pipeline(ps, c, g, depot=0, verbose=False)
        t = res["timings"]
        print(f"{label:<30} | {t['merge_sort_ms']:>10.4f} | {t['knapsack_dp_ms']:>12.4f} | {t['dijkstra_ms']:>10.4f} | {t['tsp_nn_ms']:>10.4f} | {t['total_ms']:>11.4f}")


if __name__ == "__main__":
    with open("datasets/small_dataset.json", "r") as f:
        small_data = json.load(f)

    small_parcels = [Parcel(**p) for p in small_data["parcels"]]
    small_graph = DeliveryGraph(small_data["num_vertices"])
    for u, v, w in small_data["edges"]:
        small_graph.add_edge(u, v, w)

    run_courier_pipeline(small_parcels, small_data["capacity"], small_graph, depot=0, verbose=True)
    run_benchmarks()
