
from dataclasses import dataclass, field
from typing import Dict, List, Tuple, Optional, Set

Hyperedge = Tuple[List[str], float]

@dataclass
class AndOrGraph:
    edges: Dict[str, List[Hyperedge]]              # adjacency list
    heuristic: Dict[str, float]                    # admissible heuristic for nodes
    terminals: Set[str] = field(default_factory=set)  # terminal nodes (cost=0)

@dataclass
class AOStarState:
    g: AndOrGraph
    best_edge: Dict[str, Optional[Hyperedge]] = field(default_factory=dict)  # chosen hyperedge for each node
    cost: Dict[str, float] = field(default_factory=dict)                     # current cost estimate f(n)
    solved: Dict[str, bool] = field(default_factory=dict)                    # whether node is "solved"
    expanded: Set[str] = field(default_factory=set)                           # nodes that have been expanded at least once
    iteration: int = 0

    def initialize(self, start: str):
        # init costs by heuristic or 0 for terminals
        for n in self.all_nodes():
            if n in self.g.terminals:
                self.cost[n] = 0.0
            else:
                self.cost[n] = float(self.g.heuristic.get(n, 0.0))
            self.solved[n] = n in self.g.terminals
            self.best_edge[n] = None

        # Ensuring start is known even if not in heuristic
        if start not in self.cost:
            self.cost[start] = float(self.g.heuristic.get(start, 0.0))
            self.solved[start] = start in self.g.terminals
            self.best_edge[start] = None

    def all_nodes(self) -> Set[str]:
        nodes = set(self.g.edges.keys())
        for u, hyperedges in self.g.edges.items():
            for (children, _) in hyperedges:
                nodes.update(children)
        nodes.update(self.g.heuristic.keys())
        nodes.update(self.g.terminals)
        return nodes

    # Computing minimal outgoing hyperedge cost for a node
    def best_hyperedge_and_cost(self, n: str) -> Tuple[Optional[Hyperedge], float]:
        hyperedges = self.g.edges.get(n, [])
        if not hyperedges: 
            return None, self.cost.get(n, 0.0)

        best_e = None
        best_c = float("inf")
        for (children, ecost) in hyperedges:
            total = ecost + sum(self.cost.get(c, float(self.g.heuristic.get(c, 0.0))) for c in children)
            if total < best_c:
                best_c = total
                best_e = (children, ecost)
        return best_e, best_c

    # Backward cost propagation along the current best-solution graph
    def update_costs_upward(self, start: str):
        changed = True
        while changed:
            changed = False
            # Only consider nodes reachable along current best choices
            for n in self.solution_subgraph_nodes(start):
                e, c = self.best_hyperedge_and_cost(n)
                if e is None:  # leaf or terminal
                    # Keep existing cost (terminal or heuristic)
                    continue
                # A node is solved if all children on chosen hyperedge are solved
                solved_children = all(self.solved.get(ch, False) for ch in e[0])
                prev_cost = self.cost.get(n, float("inf"))
                self.cost[n] = c
                if solved_children:
                    if not self.solved.get(n, False):
                        self.solved[n] = True
                        changed = True
                if abs(prev_cost - self.cost[n]) > 1e-9:
                    changed = True
                self.best_edge[n] = e

    def pick_frontier_node(self, start: str) -> Optional[str]:
        # DFS along best_edge choices to find first non-solved expandable node
        visited = set()
        stack = [start]
        while stack:
            n = stack.pop()
            if n in visited:
                continue
            visited.add(n)
            if not self.solved.get(n, False):
                # If it has outgoing hyperedges and hasn't been expanded, expand this first
                if self.g.edges.get(n) and n not in self.expanded:
                    return n
                # Otherwise, continue down the current best hyperedge
                e = self.best_edge.get(n)
                if e is not None:
                    children, _ = e
                    stack.extend(reversed(children))
        return None

    def solution_subgraph_nodes(self, start: str) -> Set[str]:
        result = set()
        stack = [start]
        while stack:
            n = stack.pop()
            if n in result:
                continue
            result.add(n)
            e = self.best_edge.get(n)
            if e is not None:
                children, _ = e
                stack.extend(children)
        return result

    # Pretty-print evolving partial solution
    def print_partial_solution(self, start: str):
        print(f"\nIteration {self.iteration} : Evolving Partial Solution Graph")
        def rec(n: str, prefix: str = ""):
            mark = " [SOLVED]" if self.solved.get(n, False) else ""
            cost = self.cost.get(n, float('inf'))
            print(f"{prefix}{n}  (f≈{cost:.2f}){mark}")
            e = self.best_edge.get(n)
            if e is not None:
                children, ecost = e
                print(f"{prefix}  └─ AND-edge cost={ecost:.2f} -> children:")
                for ch in children:
                    rec(ch, prefix + "     ")
        rec(start)

    def run(self, start: str, verbose: bool = True):
        self.initialize(start)

        # Initialize with best choices at start
        e, c = self.best_hyperedge_and_cost(start)
        if e is not None:
            self.best_edge[start] = e
            self.cost[start] = c

        # AO* loop
        while not self.solved.get(start, False):
            self.iteration += 1
            if verbose:
                self.print_partial_solution(start)

            # 1) Expand one frontier node on current best solution graph
            n = self.pick_frontier_node(start)
            if n is None:
                # No expandable node found; stop to prevent infinite loop
                if verbose:
                    print("No further expandable nodes found. Stopping.")
                break

            # Mark expanded
            self.expanded.add(n)

            # 2) For the expanded node, recompute its best outgoing hyperedge and update upward
            e, c = self.best_hyperedge_and_cost(n)
            if e is not None:
                self.best_edge[n] = e
                self.cost[n] = c

            # 3) Propagate costs/solved status upward along current solution
            self.update_costs_upward(start)

        self.iteration += 1
        if verbose:
            self.print_partial_solution(start)

        # Annotation: pick a non-leaf node that was not expanded because its branch became worse than the chosen alternative
        note = self.annotate_pruned_nonleaf(start)
        if note:
            print("\n[Annotation] " + note)

    def annotate_pruned_nonleaf(self, start: str) -> Optional[str]:
        sol_nodes = self.solution_subgraph_nodes(start)
        candidates = []
        for n in self.g.edges.keys():
            if n not in sol_nodes and self.g.edges.get(n):
                # non-leaf not in the final solution graph
                # If it was never expanded, it was pruned by higher-cost estimate
                candidates.append(n)
        if not candidates:
            return None

        # Choosing the one with largest difference vs its parent's chosen edge
        n = candidates[0]
        parent, parent_cost, n_est = None, None, self.cost.get(n, self.g.heuristic.get(n, 0.0))
        for p, hyperedges in self.g.edges.items():
            for (children, ecost) in hyperedges:
                if n in children:
                    # Compare parent's chosen edge cost to cost via this child
                    chosen, c_cost = self.best_hyperedge_and_cost(p)
                    alt_cost = ecost + sum(self.cost.get(ch, self.g.heuristic.get(ch, 0.0)) for ch in children)
                    if chosen is not None and (children, ecost) != chosen:
                        parent = p
                        parent_cost = c_cost
                        break
            if parent: break
        if parent:
            return (f"Expansion stopped at non-leaf node '{n}' because the algorithm selected a cheaper "
                    f"alternative at its parent '{parent}'. The parent's chosen hyperedge had cost ≈ {parent_cost:.2f}, "
                    f"whereas the alternative branch including '{n}' was estimated higher (so it was never on the "
                    f"current best solution graph).")
        else:
            return (f"Expansion stopped at non-leaf node '{n}' because its estimated cost ≈ {n_est:.2f} never "
                    f"improved enough to become part of the best solution; AO* pruned it in favor of a lower-cost branch.")

def example_graph() -> AndOrGraph:
    edges: Dict[str, List[Hyperedge]] = {
        "S": [ (["A", "B"], 2.0), (["C"], 3.0) ],
        "A": [ (["D"], 2.0), (["E", "F"], 3.0) ],
        "B": [ (["G"], 1.0) ],
        "C": [ (["H", "I"], 2.0), (["J"], 4.0) ],
    }

    # Heuristic estimates (admissible) for non-terminals before expansion
    heuristic: Dict[str, float] = {
        "S": 4.0,
        "A": 3.0,
        "B": 1.0,
        "C": 5.0,
        "D": 0.0,
        "E": 0.0,
        "F": 0.0,
        "G": 0.0,
        "H": 0.0,
        "I": 0.0,
        "J": 0.0,
    }

    terminals = {"D","E","F","G","H","I","J"}
    return AndOrGraph(edges=edges, heuristic=heuristic, terminals=terminals)

def main():
    g = example_graph()
    start = "S"
    state = AOStarState(g=g)
    state.run(start, verbose=True)

if __name__ == "__main__":
    main()
