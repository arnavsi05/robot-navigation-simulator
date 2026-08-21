"""Metrics collection and aggregation helpers for benchmarking runs."""

from __future__ import annotations

from dataclasses import asdict, dataclass, field
from pathlib import Path
from typing import List, Optional

import pandas as pd


@dataclass
class TrialResult:
    """Metrics for a single planning or simulation trial."""

    scenario: str
    algorithm: str
    trial: int
    success: bool
    planning_time_sec: float
    path_length: float
    nodes_expanded: int
    simulation_steps: int = 0
    replan_count: int = 0
    total_planning_time_sec: float = 0.0


def results_to_dataframe(results: List[TrialResult]) -> pd.DataFrame:
    return pd.DataFrame([asdict(r) for r in results])


def save_results_csv(results: List[TrialResult], path: str | Path) -> pd.DataFrame:
    df = results_to_dataframe(results)
    Path(path).parent.mkdir(parents=True, exist_ok=True)
    df.to_csv(path, index=False)
    return df


def summarize(df: "pd.DataFrame") -> "pd.DataFrame":
    """Per-(scenario, algorithm) summary: success rate, mean planning time, mean path length."""
    grouped = df.groupby(["scenario", "algorithm"]).agg(
        success_rate=("success", "mean"),
        mean_planning_time_sec=("planning_time_sec", "mean"),
        mean_path_length=("path_length", "mean"),
        mean_nodes_expanded=("nodes_expanded", "mean"),
        mean_replan_count=("replan_count", "mean"),
        trials=("trial", "count"),
    )
    return grouped.reset_index()
