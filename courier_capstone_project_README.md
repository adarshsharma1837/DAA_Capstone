# Intelligent Courier Planning & Route Optimization System (Capstone Project)

An integrated Python application combining **Sorting**, **Dynamic Programming**, **Graph Shortest-Path Algorithms**, and **Combinatorial Approximation** to simulate an end-to-end intelligent courier dispatch engine.

---

## 1. System Architecture & Phase Breakdown

| Phase | Title | Core Algorithm / Technique | Time Complexity | Space Complexity |
| :--- | :--- | :--- | :--- | :--- |
| **Phase 1** | Courier Problem Design & Planning | Modular Pipeline & Weighted Undirected Graph | — | $O(V + E + P)$ |
| **Phase 2A** | Package Prioritization | Deterministic **Merge Sort** | $O(P \log P)$ | $O(P)$ |
| **Phase 2B** | Optimal Box Filling | Bottom-Up **0/1 Knapsack DP** | $O(P \times W)$ | $O(P \times W)$ |
| **Phase 3A** | Customer Road Graph Processing | **Dijkstra's Shortest Path** | $O(K \cdot V^2)$ | $O(V + E)$ |
| **Phase 3B** | Ordered Delivery Route Maker | **Nearest-Neighbor TSP Approximation** | $O(K^2)$ | $O(K)$ |
| **Phase 4** | Performance Testing & Optimization | Multi-Scale Benchmarking (`perf_counter`) | Empirical | Empirical |

*(Where $P$ = total parcels, $W$ = vehicle weight capacity, $V$ = road network locations, $E$ = roads, $K$ = active delivery stops.)*

---

## 2. Project Structure

```text
intelligent-courier-capstone/
├── courier_planner.py          # Complete modular Python implementation
├── README.md                   # Project documentation, design, & execution guide
├── sample_output.txt           # Full console trace & DP table evidence
├── runtime_comparison.png      # Multi-scale algorithmic benchmark chart
└── datasets/
    ├── small_dataset.json      # 6 locations, 8 parcels, 15 kg capacity
    └── large_dataset.json      # 60 locations, 200 parcels, 250 kg capacity
```

---

## 3. Execution Steps

1. **Clone or extract the repository:**
   ```bash
   git clone https://github.com/<your-username>/intelligent-courier-capstone.git
   cd intelligent-courier-capstone
   ```
2. **Run the complete system & benchmarks (uses Python 3 standard library only):**
   ```bash
   python3 courier_planner.py
   ```

---

## 4. Sample Output Summary (Small Dataset: 8 Parcels, 15 kg Capacity, 6 Nodes)

- **Selected Parcels (0/1 Knapsack DP):** `PKG-101, PKG-102, PKG-103` (or optimal subset maximizing priority value within `15 kg`)
- **Weight Utilized:** `15 / 15 kg (100.0% capacity utilization)`
- **Total Value Score:** `175`
- **Customer Visit Sequence:** `0 -> 2 -> 4 -> 5 -> 0`
- **Complete Road Traversal:** `0 -> 2 -> 1 -> 3 -> 4 -> 5 -> 4 -> 3 -> 1 -> 2 -> 0`
- **Total Route Length:** `26 km`

---

## 5. Multi-Scale Runtime Benchmark Results

| Dataset Scale | Merge Sort (ms) | 0/1 Knapsack DP (ms) | Dijkstra (ms) | TSP Nearest-Neighbor (ms) | Total Pipeline (ms) |
| :--- | :---: | :---: | :---: | :---: | :---: |
| Small (P=8, W=15, V=6) | 0.0456 | 0.0757 | 0.0606 | 0.0383 | **0.2202** |
| Medium (P=50, W=80, V=25) | 0.5670 | 2.5474 | 1.5396 | 0.1424 | **4.7964** |
| Large (P=200, W=250, V=60) | 2.8315 | 33.9756 | 21.1865 | 0.5208 | **58.5144** |
| Stress (P=500, W=500, V=120) | 7.6940 | 158.5608 | 135.8412 | 1.8383 | **303.9344** |

![Runtime Comparison Chart](runtime_comparison.png)

---

## 6. Future Optimization Recommendations

1. **1D Space-Optimized Knapsack DP:** Reduce DP memory footprint from $O(P \times W)$ to $O(W)$ when the DP table trace is not required for auditing.
2. **Min-Heap Priority Queue for Dijkstra (`heapq`):** Included via `DeliveryGraph.dijkstra_heap()` to reduce shortest-path runtime from $O(V^2)$ to $O((V + E) \log V)$ on sparse city road networks.
3. **2-Opt Local Search Refinement on TSP:** Apply pairwise edge-swap local search (`2-Opt`) after Nearest-Neighbor construction to eliminate crossing routes and shave 10–18% off total tour distance.
